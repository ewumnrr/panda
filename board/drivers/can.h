// IRQs: CAN1_TX, CAN1_RX0, CAN1_SCE, CAN2_TX, CAN2_RX0, CAN2_SCE, CAN3_TX, CAN3_RX0, CAN3_SCE
#define ALL_CAN_SILENT 0xFF
#define ALL_CAN_BUT_MAIN_SILENT 0xFE
#define ALL_CAN_LIVE 0

int can_live = 0, pending_can_live = 0, can_loopback = 0, can_silent = ALL_CAN_SILENT;

// ********************* instantiate queues *********************

#define can_buffer(x, size) \
  CAN_FIFOMailBox_TypeDef elems_##x[size]; \
  can_ring can_##x = { .w_ptr = 0, .r_ptr = 0, .fifo_size = size, .elems = (CAN_FIFOMailBox_TypeDef *)&elems_##x };

can_buffer(rx_q, 0x1000)
can_buffer(tx1_q, 0x100)
can_buffer(tx2_q, 0x100)

#ifdef PANDA
  can_buffer(tx3_q, 0x100)
  can_buffer(txgmlan_q, 0x100)
  can_ring *can_queues[] = {&can_tx1_q, &can_tx2_q, &can_tx3_q, &can_txgmlan_q};
#else
  can_ring *can_queues[] = {&can_tx1_q, &can_tx2_q};
#endif

// ********************* interrupt safe queue *********************

int can_pop(can_ring *q, CAN_FIFOMailBox_TypeDef *elem) {
  int ret = 0;

  enter_critical_section();
  if (q->w_ptr != q->r_ptr) {
    *elem = q->elems[q->r_ptr];
    if ((q->r_ptr + 1) == q->fifo_size) q->r_ptr = 0;
    else q->r_ptr += 1;
    ret = 1;
  }
  exit_critical_section();

  return ret;
}

int can_push(can_ring *q, CAN_FIFOMailBox_TypeDef *elem) {
  int ret = 0;
  uint32_t next_w_ptr;

  enter_critical_section();
  if ((q->w_ptr + 1) == q->fifo_size) next_w_ptr = 0;
  else next_w_ptr = q->w_ptr + 1;
  if (next_w_ptr != q->r_ptr) {
    q->elems[q->w_ptr] = *elem;
    q->w_ptr = next_w_ptr;
    ret = 1;
  }
  exit_critical_section();
  if (ret == 0) {
#ifdef DEBUG
    puts("can_push failed! Buffer is full\n");
#endif
  }
  return ret;
}

void can_clear(can_ring *q) {
  enter_critical_section();
  q->w_ptr = 0;
  q->r_ptr = 0;
  exit_critical_section();
}

// assign CAN numbering
// bus num: Can bus number on ODB connector. Sent to/from USB
//    Min: 0; Max: 127; Bit 7 marks message as receipt (bus 129 is receipt for but 1)
// cans: Look up MCU can interface from bus number
// can number: numeric lookup for MCU CAN interfaces (0 = CAN1, 1 = CAN2, etc);
// bus_lookup: Translates from 'can number' to 'bus number'.
// can_num_lookup: Translates from 'bus number' to 'can number'.
// can_forwarding: Given a bus num, lookup bus num to forward to. -1 means no forward.

int can_rx_cnt = 0;
int can_tx_cnt = 0;
int can_txd_cnt = 0;
int can_err_cnt = 0;

// NEO:         Bus 1=CAN1   Bus 2=CAN2
// Panda:       Bus 0=CAN1   Bus 1=CAN2   Bus 2=CAN3
#ifdef PANDA
  CAN_TypeDef *cans[] = {CAN1, CAN2, CAN3};
  uint8_t bus_lookup[] = {0,1,2};
  uint8_t can_num_lookup[] = {0,1,2,-1};
  int8_t can_forwarding[] = {-1,-1,-1,-1};
  uint32_t can_speed[] = {5000, 5000, 5000, 333};
  #define CAN_MAX 3
#else
  CAN_TypeDef *cans[] = {CAN1, CAN2};
  uint8_t bus_lookup[] = {1,0};
  uint8_t can_num_lookup[] = {1,0};
  int8_t can_forwarding[] = {-1,-1};
  uint32_t can_speed[] = {5000, 5000};
  #define CAN_MAX 2
#endif

#define CANIF_FROM_CAN_NUM(num) (cans[num])
#define BUS_NUM_FROM_CAN_NUM(num) (bus_lookup[num])
#define CAN_NUM_FROM_BUS_NUM(num) (can_num_lookup[num])

// other option
/*#define CAN_QUANTA 16
#define CAN_SEQ1 13
#define CAN_SEQ2 2*/

// this is needed for 1 mbps support
#define CAN_QUANTA 8
#define CAN_SEQ1 6 // roundf(quanta * 0.875f) - 1;
#define CAN_SEQ2 1 // roundf(quanta * 0.125f);

#define CAN_PCLK 24000
// 333 = 33.3 kbps
// 5000 = 500 kbps
#define can_speed_to_prescaler(x) (CAN_PCLK / CAN_QUANTA * 10 / (x))

void process_can(uint8_t can_number);

void can_init(uint8_t can_number) {
  if (can_number == 0xff) return;

#ifdef DEBUG
  puts("can_init can_number "); puth(can_number); puts("\n");
  puts("can_init can_loopback "); puth(can_loopback); puts("\n");
  puts("can_init can_silent "); puth(can_silent); puts("\n");
#endif

  CAN_TypeDef *CAN = CANIF_FROM_CAN_NUM(can_number);
  set_can_enable(CAN, 1);

  // The software initialization can be done while the hardware
  // is in Initialization mode. To enter this mode the software
  // sets the INRQ bit in the CAN_MCR register and waits until
  // the hardware has confirmed the request by setting the
  // INAK bit in the CAN_MSR register.

  // TTCM = Time Triggered Communication Mode
  //        (Time stamp receive and sent messages)
  CAN->MCR = CAN_MCR_TTCM | CAN_MCR_INRQ;
  while((CAN->MSR & CAN_MSR_INAK) != CAN_MSR_INAK);

  // BTR = Bit Timig Register
  // look at the bits at the right time
  // set time quanta from defines
  CAN->BTR = (CAN_BTR_TS1_0 * (CAN_SEQ1-1)) |
             (CAN_BTR_TS2_0 * (CAN_SEQ2-1)) |
             (can_speed_to_prescaler(can_speed[BUS_NUM_FROM_CAN_NUM(can_number)]) - 1);

  // silent loopback mode for debugging
  // SIL = silent mode (no transmit)
  // LBK = loopback (no external receive, just receiving own sent messages)
  // SIL + LBK = (internal loopback, nothing is sent or received to/from outside)
  if (can_loopback) {
    CAN->BTR |= CAN_BTR_SILM | CAN_BTR_LBKM;
  }

  if (can_silent & (1 << can_number)) {
    CAN->BTR |= CAN_BTR_SILM;
  }

  // reset
  // leave init mode by clearing INRQ
  // leave time stamps on
  CAN->MCR = CAN_MCR_TTCM | CAN_MCR_ABOM;

  #define CAN_TIMEOUT 1000000
  int tmp = 0;
  while((CAN->MSR & CAN_MSR_INAK) == CAN_MSR_INAK && tmp < CAN_TIMEOUT) tmp++;

  if (tmp == CAN_TIMEOUT) {
#ifdef DEBUG
    puts("CAN init FAILED!!!!!\n");
    puth(can_number); puts(" ");
    puth(BUS_NUM_FROM_CAN_NUM(can_number)); puts("\n");
#endif
  }

  // filter for not having to receive all
  // messages
  // set FINIT bit so the filter can be
  // modified / set up
  CAN->FMR |= CAN_FMR_FINIT;

  // no mask, all messages received
  CAN->sFilterRegister[0].FR1 = 0;
  CAN->sFilterRegister[0].FR2 = 0;
  CAN->sFilterRegister[14].FR1 = 0;
  CAN->sFilterRegister[14].FR2 = 0;
  CAN->FA1R |= 1 | (1 << 14);

  // done programming the filters, commit change
  CAN->FMR &= ~(CAN_FMR_FINIT);

  // enable certain CAN interrupts
  // Transmit mailbox empty interrupt enable (RQCPx bit is set)
  // FIFO message pending interrupt enable (FMP[1:0] bits are not 0) 
  CAN->IER = CAN_IER_TMEIE | CAN_IER_FMPIE0;

  switch (can_number) {
    case 0:
      NVIC_EnableIRQ(CAN1_TX_IRQn);  // Send Interrupt
      NVIC_EnableIRQ(CAN1_RX0_IRQn); // Receive Interrupt
      NVIC_EnableIRQ(CAN1_SCE_IRQn); // Status Change Error Interrupt
      break;
    case 1:
      NVIC_EnableIRQ(CAN2_TX_IRQn);
      NVIC_EnableIRQ(CAN2_RX0_IRQn);
      NVIC_EnableIRQ(CAN2_SCE_IRQn);
      break;
#ifdef CAN3
    case 2:
      NVIC_EnableIRQ(CAN3_TX_IRQn);
      NVIC_EnableIRQ(CAN3_RX0_IRQn);
      NVIC_EnableIRQ(CAN3_SCE_IRQn);
      break;
#endif
  }

  // in case there are queued up messages
  process_can(can_number);
}

void can_init_all() {
  for (int i=0; i < CAN_MAX; i++) {
    can_init(i);
  }
}

void can_set_gmlan(int bus) {
  #ifdef PANDA
  if (bus == -1 || bus != can_num_lookup[3]) {
    // GMLAN OFF
    switch (can_num_lookup[3]) {
      case 1:
#ifdef DEBUG
        puts("disable GMLAN on CAN2\n");
#endif
        set_can_mode(1, 0);
        bus_lookup[1] = 1;
        can_num_lookup[1] = 1;
        can_num_lookup[3] = -1;
        can_init(1);
        break;
      case 2:
#ifdef DEBUG
        puts("disable GMLAN on CAN3\n");
#endif
        set_can_mode(2, 0);
        bus_lookup[2] = 2;
        can_num_lookup[2] = 2;
        can_num_lookup[3] = -1;
        can_init(2);
        break;
    }
  }

  if (bus == 1) {
#ifdef DEBUG
    puts("GMLAN on CAN2\n");
#endif
    // GMLAN on CAN2
    set_can_mode(1, 1);
    bus_lookup[1] = 3;
    can_num_lookup[1] = -1;
    can_num_lookup[3] = 1;
    can_init(1);
  } else if (bus == 2 && revision == PANDA_REV_C) {
#ifdef DEBUG
    puts("GMLAN on CAN3\n");
#endif
    // GMLAN on CAN3
    set_can_mode(2, 1);
    bus_lookup[2] = 3;
    can_num_lookup[2] = -1;
    can_num_lookup[3] = 2;
    can_init(2);
  }
  #endif
}

// CAN error
void can_sce(CAN_TypeDef *CAN) {
  can_err_cnt += 1;
  #ifdef DEBUG
    if (CAN==CAN1) puts("CAN1:  ");
    if (CAN==CAN2) puts("CAN2:  ");
    #ifdef CAN3
      if (CAN==CAN3) puts("CAN3:  ");
    #endif
    puts("MSR:");
    puth(CAN->MSR);
    puts(" TSR:");
    puth(CAN->TSR);
    puts(" RF0R:");
    puth(CAN->RF0R);
    puts(" RF1R:");
    puth(CAN->RF1R);
    puts(" ESR:");
    puth(CAN->ESR);
    puts("\n");
  #endif

  // clear current send
  CAN->TSR |= CAN_TSR_ABRQ0;
  CAN->MSR = CAN->MSR;
}

// ***************************** CAN *****************************

void process_can(uint8_t can_number) {
  static int did_send_once=0;
  int did_send=0;

  if (can_number == 0xff) return;

  // note: this driver is only using transmit mailbox 0, so all bits hard coded for mailbox 0 (xxx0)

  enter_critical_section();

  CAN_TypeDef *CAN = CANIF_FROM_CAN_NUM(can_number);
  uint8_t bus_number = BUS_NUM_FROM_CAN_NUM(can_number);
  #ifdef DEBUG
    puts("Tx "); puth(can_number); puts("\n");
  #endif


  // Note: FIFO = HW receive buffer, MailBox = HW send buffer

//  puts("MCR:");
//  puth(CAN->MCR);
//  puts(" MSR:");
//  puth(CAN->MSR);
//  puts(" TSR:");
//  puth(CAN->TSR);
//  puts(" RF0R:");
//  puth(CAN->RF0R);
//  puts(" RF1R:");
//  puth(CAN->RF1R);
#ifdef DEBUG
  puts("ESR:");
  puth(CAN->ESR);
  puts("\n");
#endif

  // check for bus has gone into passive mode:
  if ((CAN->ESR & CAN_ESR_EPVF) == CAN_ESR_EPVF) {
#ifdef DEBUG
    puts("Bus is passive!!!\n");
#endif
  }

  if (did_send_once) {
    // error bits get set even if transaction didn't complete
    if ((CAN->TSR & CAN_TSR_TERR0) == CAN_TSR_TERR0) {
      #ifdef DEBUG
        puts("CAN TX ERROR!\n");
      #endif
    }

    if ((CAN->TSR & CAN_TSR_ALST0) == CAN_TSR_ALST0) {
      #ifdef DEBUG
        puts("CAN TX ARBITRATION LOST!\n");
      #endif
    }

    if ((CAN->TSR & CAN_TSR_RQCP0) == CAN_TSR_RQCP0) {
    // check for errors if completion flag is set
//        puts("previous transmit ended\n");
      if ((CAN->TSR & CAN_TSR_TXOK0) == CAN_TSR_TXOK0) {
#ifdef DEBUG
        puts("prev Tx OK\n");
#endif
        can_txd_cnt += 1;

#if 0
        puts("making a copy if what was sent into can_rx_q queue\n");
        CAN_FIFOMailBox_TypeDef to_push;

        // RIR  CAN receive FIFO mailbox identifier register */
        // TIR  CAN TX mailbox identifier register */
        to_push.RIR = CAN->sTxMailBox[0].TIR;


        // RDTR CAN receive FIFO mailbox data length control and time stamp register */
        // TDTR CAN mailbox data length control and time stamp register */
        to_push.RDTR = (CAN->sTxMailBox[0].TDTR & 0xFFFF000F) | ((CAN_BUS_RET_FLAG | bus_number) << 4);

        // RDLR CAN receive FIFO mailbox data low register */
        // TDLR CAN mailbox data low register */
        to_push.RDLR = CAN->sTxMailBox[0].TDLR;

        // RDHR CAN receive FIFO mailbox data high register */
        // TDHR CAN mailbox data high register */
        to_push.RDHR = CAN->sTxMailBox[0].TDHR;
        can_push(&can_rx_q, &to_push);
#endif
      } else { // last transmit was flagged not OK
#ifdef DEBUG
          puts("prev Tx NOT OK!!\n");
#endif
      }
    }  else { // transaction did not complete
#ifdef DEBUG
        puts("prev Tx pending\n");
#endif
    }
  } // did send once

  // check for empty send mailbox means transaction has completed one way or the other
  if ((CAN->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
#ifdef DEBUG
    puts("good, mbox empty\n");
#endif

    // check if we have a message to send
    CAN_FIFOMailBox_TypeDef to_send;
    if (can_pop(can_queues[bus_number], &to_send)) {
#ifdef DEBUG
      puts("sending message\n");
#endif
      can_tx_cnt += 1;
      CAN->sTxMailBox[0].TDLR = to_send.RDLR;
      CAN->sTxMailBox[0].TDHR = to_send.RDHR;
      CAN->sTxMailBox[0].TDTR = to_send.RDTR;
      CAN->sTxMailBox[0].TIR = to_send.RIR;        // this will have the send request bit set (TXRQ)
      did_send_once=1;
      did_send=1;
    } else {
#ifdef DEBUG
      puts("Nothing to send\n");
#endif
    }
  } else {
#ifdef DEBUG
    puts("transmit mbox in use\n");
#endif
  }
  if (!did_send){
      // clear interrupt
      // RQCP is set by hardware when the last request (transmit or abort) has been performed.
      // Cleared by software writing a “1” or by hardware on transmission request (TXRQ2 set in CAN_TMID2R register).
      // Clearing this bit clears all the status bits (TXOK, ALST and TERR)
      // we did not send but need to clear status change interrupt
      CAN->TSR |= CAN_TSR_RQCP0;
  }
  exit_critical_section();
}

// CAN receive handlers
// blink blue when we are receiving CAN messages
void can_rx(uint8_t can_number) {
  CAN_TypeDef *CAN = CANIF_FROM_CAN_NUM(can_number);
  uint8_t bus_number = BUS_NUM_FROM_CAN_NUM(can_number);
#ifdef DEBUG
  puts("Rx "); puth(can_number); puts("\n");
#endif
  while (CAN->RF0R & CAN_RF0R_FMP0) {
    can_rx_cnt += 1;

    // can is live
    pending_can_live = 1;

    // add to my fifo
    CAN_FIFOMailBox_TypeDef to_push;
    to_push.RIR = CAN->sFIFOMailBox[0].RIR;
    to_push.RDTR = CAN->sFIFOMailBox[0].RDTR;
    to_push.RDLR = CAN->sFIFOMailBox[0].RDLR;
    to_push.RDHR = CAN->sFIFOMailBox[0].RDHR;

    // Chevy Volt setup with Panda CAN forwarding:
    // Panda is plugged into OBDII port and forwards some messages from
    // powertrain CAN bus to object bus, and messages that openpilot sends
    // on object bus to either powertrain, or to chassis bus.
    #ifdef PANDA
      uint32_t addr;
      if (to_push.RIR & 4) {
        // Extended
        addr = to_push.RIR >> 3;
      } else {
        // Normal
        addr = to_push.RIR >> 21;
      }

      int dst_can_idx = -1; // default = no forward

      // Received on Object CAN, bus=1 (from Voltboard VT)
      // 0x180 (384)  = LKA Steering command
      // 0x409 (1033) = ASCM Keep Alive
      // 0x2cb (715)  = Gas Regen Command
      // 0x370 (880)  = Cruise Control Status
      if (bus_number == 1 && (addr == 0x180 || addr == 0x409 || addr == 0x2cb || addr == 0x370)) {
        dst_can_idx = 0; // Sent to Powertrain CAN

      // Received on Object CAN, bus=1 (from Voltboard VT)
      // 0x315 (789) = Friction Brake command (normally Chassis CAN bus command)
      } else if (bus_number == 1 && addr == 0x315) {
        dst_can_idx = 2; // Sent to Chassis CAN

      // Received on Powertrain CAN, bus=0
      // 189 = Regen Paddle
      // 241 = Brake Pedal Position
      // 298 = Door Status
      // 309 = Park/Neutral/Drive/Reverse
      // 320 = Turn Signals
      // 388 = Hands off steering detection / Torque status
      // 481 = Steering wheel buttons
      // 485 = Steering wheel angle
      // 840 = Wheel speed (front)
      // 842 = Wheel speed (rear)
      } else if (bus_number == 0 && (addr == 189 || addr == 241 || addr == 298 || addr == 309 || addr == 320 || addr == 388 || addr == 417 || addr == 481 || addr == 485 || addr == 840 || addr == 842)) {
        dst_can_idx = 1; // Sent to Object CAN (to Voltboard VT)
#ifdef DEBUG
        puts("msg ");
        puth(addr);
        puts(" -> Voltboard bus 1\n");
#endif
      }
      if (dst_can_idx != -1) {
        CAN_FIFOMailBox_TypeDef to_send;
        to_send.RIR = to_push.RIR | 1; // TXRQ
        to_send.RDTR = to_push.RDTR;
        to_send.RDLR = to_push.RDLR;
        to_send.RDHR = to_push.RDHR;
        can_send(&to_send, dst_can_idx);
      }
    #endif

    // modify RDTR for our API
    to_push.RDTR = (to_push.RDTR & 0xFFFF000F) | (bus_number << 4);
    safety_rx_hook(&to_push);

    #ifdef PANDA
      set_led(LED_BLUE, 1);
    #endif

#if 0
    can_push(&can_rx_q, &to_push);
#endif

    // release FIFO, so it can be reused for incoming packets
    // otherwise packets will be dropped
    CAN->RF0R |= CAN_RF0R_RFOM0;
  }
}

void CAN1_TX_IRQHandler() { process_can(0); }
void CAN1_RX0_IRQHandler() { can_rx(0); }
void CAN1_SCE_IRQHandler() { can_sce(CAN1); }

void CAN2_TX_IRQHandler() { process_can(1); }
void CAN2_RX0_IRQHandler() { can_rx(1); }
void CAN2_SCE_IRQHandler() { can_sce(CAN2); }

#ifdef CAN3
void CAN3_TX_IRQHandler() { process_can(2); }
void CAN3_RX0_IRQHandler() { can_rx(2); }
void CAN3_SCE_IRQHandler() { can_sce(CAN3); }
#endif

void can_send(CAN_FIFOMailBox_TypeDef *to_push, uint8_t bus_number) {
#ifdef DEBUG
  puts("can_send "); puth(bus_number); puts("\n");
#endif
  if ((bus_number==1)  || safety_tx_hook(to_push)) {
    if (bus_number==1) {
#ifdef DEBUG
      puts("safety: Bus 1\n");
#endif
    } else {
#ifdef DEBUG
      puts("safety: OK\n");
#endif
    }
    if (bus_number < BUS_MAX) {
      // add CAN packet to send queue
      // bus number isn't passed through
      to_push->RDTR &= 0xF;
      can_push(can_queues[bus_number], to_push);
      process_can(CAN_NUM_FROM_BUS_NUM(bus_number));
    }
  } else {
#ifdef DEBUG
    puts("safety:NOT OK\n");
#endif
  }
}

void can_set_forwarding(int from, int to) {
  can_forwarding[from] = to;
}
