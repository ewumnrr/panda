// board enforces
//   in-state
//      accel set/resume
//   out-state
//      cancel button
//      regen paddle
//      accel rising edge
//      brake rising edge
//      brake > 0mph

#ifdef DEBUG
int puts(const char *a);
void puth(unsigned int i);
#endif

// gm_: poor man's namespacing
int gm_brake_prev = 0;
int gm_gas_prev = 0;
int gm_speed = 0;

// silence everything if stock ECUs are still online
int gm_ascm_detected = 0;

static void gm_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {

  uint32_t addr;
  if (to_push->RIR & 4) {
    // Extended
    // Not looked at, but have to be separated
    // to avoid address collision
    addr = to_push->RIR >> 3;
  } else {
    // Normal
    addr = to_push->RIR >> 21;
  }

  // sample speed, really only care if car is moving or not
  // rear left wheel speed
  if (addr == 842) {
    gm_speed = to_push->RDLR & 0xFFFF;
  }

  // check if stock ASCM ECU is still online
  int bus_number = (to_push->RDTR >> 4) & 0xFF;
  if (bus_number == 0 && addr == 715) {
    gm_ascm_detected = 1;
    controls_allowed = 0;
  }

  // ACC steering wheel buttons
  if (addr == 481) {
    int buttons = (to_push->RDHR >> 12) & 0x7;
    // res/set - enable, cancel button - disable
    if (buttons == 2 || buttons == 3) {
      controls_allowed = 1;
#ifdef DEBUG
      puts("buttons=2 or 3, controls_allowed=1\n");
#endif
    } else if (buttons == 6) {
      controls_allowed = 0;
#ifdef DEBUG
      puts("buttons=6, controls_allowed=0\n");
#endif
    }
  }

  // exit controls on rising edge of brake press or on brake press when
  // speed > 0
  if (addr == 241) {
    int brake = (to_push->RDLR & 0xFF00) >> 8;
    // Brake pedal's potentiometer returns near-zero reading
    // even when pedal is not pressed
    if (brake < 10) {
      brake = 0;
    }
    if (brake && (!gm_brake_prev || gm_speed)) {
       controls_allowed = 0;
#ifdef DEBUG
       puts("brake newly applied or brake applied and speed no longer 0, controls_allowed=0 \n");
#endif
    }
    gm_brake_prev = brake;
  }

  // exit controls on rising edge of gas press
  if (addr == 417) {
    int gas = to_push->RDHR & 0xFF0000;
    if (gas && !gm_gas_prev) {
      controls_allowed = 0;
#ifdef DEBUG
      puts("gas newly applied, controls_allowed=0\n");
#endif
    }
    gm_gas_prev = gas;
  }

  // exit controls on regen paddle
  if (addr == 189) {
    int regen = to_push->RDLR & 0x20;
    if (regen) {
      controls_allowed = 0;
#ifdef DEBUG
      puts("regen paddle applied, controls_allowed=0\n");
#endif
    }
  }
}

// all commands: gas/regen, friction brake and steering
// if controls_allowed and no pedals pressed
//     allow all commands up to limit
// else
//     block all commands that produce actuation

static int gm_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {

  // There can be only one! (ASCM)
  if (gm_ascm_detected) {
#ifdef DEBUG
    puts("gm_ascm_detected, exiting gm_tx_hook\n");
#endif
    return 0;
  }

  // disallow actuator commands if gas or brake (with vehicle moving) are pressed
  // and the the latching controls_allowed flag is True
  int pedal_pressed = gm_gas_prev || (gm_brake_prev && gm_speed);
  int current_controls_allowed = controls_allowed && !pedal_pressed;

  uint32_t addr;
  if (to_send->RIR & 4) {
    // Extended
    addr = to_send->RIR >> 3;
  } else {
    // Normal
    addr = to_send->RIR >> 21;
  }

  // BRAKE: safety check
  if (addr == 789) {
    int rdlr = to_send->RDLR;
    int brake = ((rdlr & 0xF) << 8) + ((rdlr & 0xFF00) >> 8);
    brake = (0x1000 - brake) & 0xFFF;
    if (current_controls_allowed) {
#ifdef DEBUG
      puts("brake safety check, exiting\n");
#endif
      if (brake > 255) return 0;
    } else {
#ifdef DEBUG
      puts("brake safety check, exiting\n");
#endif
      if (brake != 0) return 0;
    }
  }

  // LKA STEER: safety check
  if (addr == 384) {
    int rdlr = to_send->RDLR;
    int steer = ((rdlr & 0x7) << 8) + ((rdlr & 0xFF00) >> 8);
    int max_steer = 255;
    if (current_controls_allowed) {
      // Signed arithmetic
      if (steer & 0x400) {
#ifdef DEBUG
        puts("steering safety check, exiting\n");
#endif
        if (steer < (0x800 - max_steer)) return 0;
      } else {
#ifdef DEBUG
        puts("steering safety check, exiting\n");
#endif
        if (steer > max_steer) return 0;
      }
    } else {
#ifdef DEBUG
      puts("steering safety check, exiting\n");
#endif
      if (steer != 0) return 0;
    }
  }

  // PARK ASSIST STEER: unlimited torque, no thanks
  if (addr == 823){
#ifdef DEBUG
     puts("park assist, exiting\n");
#endif
     return 0;
  }

  // GAS/REGEN: safety check
  if (addr == 715) {
    int rdlr = to_send->RDLR;
    int gas_regen = ((rdlr & 0x7F0000) >> 11) + ((rdlr & 0xF8000000) >> 27);
    int apply = rdlr & 1;
    if (current_controls_allowed) {
      if (gas_regen > 3072) {
#ifdef DEBUG
        puts("regen safety check, exiting\n");
#endif
        return 0;
      }
    } else {
      // Disabled message is !engaed with gas
      // value that corresponds to max regen.
      if (apply || gas_regen != 1404) {
#ifdef DEBUG
         puts("regen safety check, exiting\n");
#endif
         return 0;
      }
    }
  }

  // 1 allows the message through
  return true;
}

static int gm_tx_lin_hook(int lin_num, uint8_t *data, int len) {
  // LIN is not used in Volt
  return false;
}

static void gm_init(int16_t param) {
  controls_allowed = 0;
}

const safety_hooks gm_hooks = {
  .init = gm_init,
  .rx = gm_rx_hook,
  .tx = gm_tx_hook,
  .tx_lin = gm_tx_lin_hook,
};

