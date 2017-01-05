#include "rilmac.h"
#include "random.h"
#include "time.h"
#include "parent.h"

static class RILMacClass : public TclClass {
public:
  RILMacClass() : TclClass("Mac/RILMac") {}
  TclObject* create(int, const char*const*) {
    return new RILMac();
  }
} class_RILMac;


RILMac::RILMac() : Mac(), config_(),
  backoffTimer(this), senddoneTimer(this), recvdoneTimer(this), retransmitTimer(this),
  wakeupTimer(this), radioOffTimer(this), TsTimer(this)
{
    rx_state_ = tx_state_ = MAC_IDLE;
    sender_cw_ = my_cw_ = config_.cw_min;
    pktRx_ = pktTx_ = pktRRTS_ = pktACK_ = 0;
    dataRetry_ = 0;
    phi_ = 0.01;
    Ts_ = 0;
    tx_lifetime_ = 9999999; tx_delay_ = 0; outgoing_rate_ = 1; incoming_rate_ = 1;
    send_scheduledSleep_ = recv_scheduledSleep_ = 0;
    running_mac_mode_ = RIMAC;
    total_outgoing_ = 1;
    curRecv = 0;
    lastUpdate_ = 0; lastUpdateCount_ = 0;last_increase_time_=last_decrease_time_=0;
    wakeupTimer.start((Random::uniform()*1.1));
    bind("runningmac", &running_mac_mode_);
    int tempd=0; bind("e2edelayreq", &tempd); e2e_delay_req_ = tempd;
    bind("defaulttr",&defaulttr_);
    Tr_ = defaulttr_;
}


/*------------------ misc functions ------------------------*/
// set or get mac type, used for all messages
inline u_int16_t RILMac::hdr_mactype(char *hdr, u_int16_t type) {
  struct hdr_mac *dh = (struct hdr_mac*) hdr;
  if (type) dh->ftype_ = (MacFrameType)type;
  return dh->ftype_;
}

// set or get submac type, used for all messages
inline u_int16_t RILMac::hdr_submactype(char *hdr, u_int16_t type) {
  struct hdr_mac *dh = (struct hdr_mac*) hdr;
  if (type) {dh->hdr_type_ &= 0x8000; dh->hdr_type_ |= (0x7fff&type);}
  return dh->hdr_type_&0x7fff;
}

// set/get Tr, bits 1-16
inline u_int16_t RILMac::hdr_tr(char* hdr, int32_t value) {
  struct hdr_mac *dh = (struct hdr_mac*) hdr;
  if (value > -1) {
    dh->padding_ &= 0x0000ffff;
    u_int32_t tempvalue = value&0x0000ffff;
    dh->padding_ |= (tempvalue<<16UL);
  }
  return (dh->padding_&0xffff0000)>>16UL;
}

// set/get phi, bits 16-32
inline u_int16_t RILMac::hdr_phi(char *hdr, int32_t value) {
  struct hdr_mac *dh = (struct hdr_mac*) hdr;
  if (value > -1) {
    dh->padding_ &= 0xffff0000;
    u_int32_t tempvalue = value&0x0000ffff;
    dh->padding_ |= (tempvalue);
  }
  return (dh->padding_&0x0000ffff);
}

// set/get lifetime
inline u_int16_t RILMac::hdr_lifetime(char *hdr, int32_t value) {
  struct hdr_mac *dh = (struct hdr_mac*) hdr;
  if (value > -1) {
      dh->lifetime_ = value;
  }
  return (dh->lifetime_);
}

// set/get delay
inline u_int16_t RILMac::hdr_delay(char *hdr, int32_t value) {
  struct hdr_mac *dh = (struct hdr_mac*) hdr;
  if (value > -1) {
    dh->delay_ = value;
  }
  return (dh->delay_);
}

// set/get backoff window
inline u_int16_t RILMac::hdr_backoff(char *hdr, int32_t value) {
  struct hdr_mac *dh = (struct hdr_mac*) hdr;
  if (value > -1) {
    dh->backoff_ = value;
  }
  return (dh->backoff_);
}

// set/get data rate
inline u_int16_t RILMac::hdr_rate(char* hdr, int32_t value) {
  struct hdr_mac *dh = (struct hdr_mac*) hdr;
  if (value > -1) {
    dh->backoff_ = value;
  }
  return (dh->backoff_);
}

// set/get arrivaltime
inline double RILMac::hdr_arrivaltime(char* hdr, double value) {
  struct hdr_mac *dh = (struct hdr_mac*) hdr;
  if (value > 0) {
    dh->arrivaltime_ = value;
  }
  return (dh->arrivaltime_);
}

// set/get arrivaltime
inline u_int16_t RILMac::hdr_minid(char* hdr, int32_t value) {
  struct hdr_mac *dh = (struct hdr_mac*) hdr;
  if (value > -1) {
    dh->minid_ = value;
  }
  return (dh->minid_);
}

inline u_int16_t RILMac::hdr_hop(char* hdr, int32_t value) {
  struct hdr_mac *dh = (struct hdr_mac*) hdr;
  if (value > -1) {
    dh->hop_ = value;
  }
  return (dh->hop_);
}

// get the minimum lifetime of my senders
double RILMac::updateMySenderlifetime(int src, double lifetime, double rate, double delay) {
  bool found = false;
  double min = 9999999;
  for (uint32_t i=0;i<senderTable.size();i++) {
    if (senderTable.at(i).id == src) {
      senderTable.at(i).lifetime = lifetime;
      senderTable.at(i).rate = rate;
      senderTable.at(i).delay = delay;
      senderTable.at(i).lastUpdate = NOW;
      senderTable.at(i).invalid = false;
      found = true;
    }
    if (God::instance()->current_parent[senderTable.at(i).id]!=index_) {
        senderTable.at(i).invalid = true;
    }
    if (senderTable.at(i).lifetime < min && !senderTable.at(i).invalid) {
        min = senderTable.at(i).lifetime;
    }

  }
  if (!found) {
    senderEntry n(src, lifetime, rate, Tr_, phi_, delay);
    senderTable.push_back(n);
    if (lifetime < min) {
        min = lifetime;
    }
  }
  if (min + 0.001 > 9999999)
  {
    min = newnetif()->getLifetime();
  }
  return min;
}

receiverEntry* RILMac::getMyReceiverInfo(int id) {
    receiverEntry* e = 0;
    for (uint32_t i=0;i<receiverTable.size();i++) {
        if (receiverTable.at(i).id == id) {
            e = &receiverTable.at(i);
        }
    }
    return e;
}

void RILMac::updateMyReceiverInfo(int id, int minid, int hop, double lifetime, double tr, double phi, double ts, double delay) {
    bool found = false;
    for (uint32_t i=0;i<receiverTable.size();i++) {
        if (receiverTable.at(i).id == id) {
            receiverTable.at(i).minid = minid;
            receiverTable.at(i).hop = hop;
            receiverTable.at(i).Tr = tr;
            receiverTable.at(i).phi = phi;
            receiverTable.at(i).Ts = ts;
            receiverTable.at(i).delay = delay;
            receiverTable.at(i).lifetime = lifetime;
            found = true;
            curRecv = &receiverTable.at(i);
            break;
        }
    }
    if (!found) {
        receiverEntry n(id, minid, lifetime, tr, phi, ts,delay);
        receiverTable.push_back(n);
        curRecv = &receiverTable.back();
        curRecv->hop = hop;
    }
}

void RILMac::dumpStatus() {
    printf("node %d tx_state_ %d rx_state_ %d backoff %d data %d ack %d beacon %d\n",index_,tx_state_,rx_state_,backoffTimer.busy(), pktTx_!=NULL,pktACK_!=NULL,pktRRTS_!=NULL);
}

// no ongoing transaction
bool RILMac::noOngoingTransaction(){
  return (tx_state_ == MAC_IDLE && rx_state_ == MAC_IDLE
          && !retransmitTimer.busy() && !backoffTimer.busy());
}

// get data tx time
double RILMac::txtime(Packet *p) {
  struct hdr_cmn *ch = HDR_CMN(p);
  double t = ch->txtime();
  if (t < 0.0) t = 0.0;
  return t;
}

/*-------------------- functions related to recv ----------------------*/
// first entry to handle recv packet
void RILMac::recv(Packet *p, Handler *h) {
  struct hdr_cmn *hdr = HDR_CMN(p);
  if (hdr->direction() == hdr_cmn::DOWN) {
    // first handle outgoing data
    txHandler_ = h;
    h->handle((Event*) 0);
    receiverEntry* e = getMyReceiverInfo(hdr_dst((char*)HDR_MAC(p)));
    if (e!=0) {
        if (pktTx_!=NULL) {
            (e->macqueue).push_back(p);
        }
        else {
            pktTx_ = p;
        }
        curRecv = e;
    }
    else {
        receiverEntry n(hdr_dst((char*)HDR_MAC(p)), index_,newnetif()->getLifetime(),defaulttr_, defaulttr_);
        receiverTable.push_back(n);
        curRecv = &receiverTable.back();
        if (pktTx_!=NULL) {
            receiverTable.back().macqueue.push_back(p);
        }
        else {
            pktTx_ = p;
        }
    }
    // temp to send data immediately
    hdr_arrivaltime((char*)HDR_MAC(pktTx_), NOW);
    tempToSendData(0, index_);
  }
  else {
      // then handle incoming data
      if(tx_state_ != MAC_IDLE && hdr->error() == 0) {
        hdr->error() = 1;
      }
      if (rx_state_ == MAC_IDLE) {
        rx_state_ = MAC_RECV;
        pktRx_ = p;
        recvdoneTimer.start(txtime(p));
      }
      else {
        if (pktRx_->txinfo_.RxPr / p->txinfo_.RxPr >= p->txinfo_.CPThresh) {
          // power too low, ignore the incoming packet, dont change mac state
          Packet::free(p);
        }
        else {
          // power is high enough to cause collision
          rx_state_ = MAC_COLL;
          // decide which packets to be left
          if (txtime(p) > recvdoneTimer.expire()) {
            recvdoneTimer.stop();
            Packet::free(pktRx_); pktRx_ = p;
            recvdoneTimer.start(txtime(pktRx_));
          }
          else {
            //ignore the incoming packet
            Packet::free(p);
          }
        }
      }
  }
}

// recv timer handler
void RILMac::recvdoneHandler() {
  if (pktRx_==NULL || (rx_state_ == MAC_IDLE)) {
    printf("error_y!!! node %d wrong state in recvHandle\n",index_);
    exit(-1);
  }

  hdr_cmn *ch = HDR_CMN(pktRx_);
  // detect collision
  if (tx_state_ != MAC_IDLE || rx_state_ == MAC_COLL || ch->error()) {
    Packet::free(pktRx_); pktRx_ = NULL; rx_state_ = MAC_IDLE;
    if (running_mac_mode_ != XMAC) inc_sender_cw();
    double minon = sender_cw_*config_.backoff_slot+128*8.0/config_.data_rate+config_.CCA;
    tempCloseRadio(minon);
    return;
  }

  char* mh = (char*)HDR_MAC(pktRx_);
  int dst = hdr_dst(mh);
  u_int16_t  type = hdr_mactype(mh);
  u_int16_t  subtype = hdr_submactype(mh);

  // this is an ack to some node's data packet, take it as a receiver's beacon
  if (dst != index_ && (MacFrameType)type == MF_CONTROL && subtype == MAC_RIL_ACK){
    // take this one as receiver's beacon
    subtype = MAC_RIL_RRTS;
    hdr_submactype((char*)HDR_MAC(pktRx_), subtype);
    dst = MAC_BROADCAST;
    hdr_dst((char*)HDR_MAC(pktRx_), dst);
  }

  // this is neither a packet to me nor a bcast packet, drop it
  if (dst != index_ && dst != BCAST_ADDR) {
    Packet::free(pktRx_); pktRx_ = NULL; rx_state_ = MAC_IDLE;
    tempCloseRadio(0);
    return;
  }
  switch(type) {
      case MF_CONTROL:
        switch(subtype) {
        // receive an ack
        case MAC_RIL_ACK:
          recvACK(pktRx_);
          break;
        // receive a receiver's beacon
        case MAC_RIL_RRTS:
          recvBeaconRRTS(pktRx_);
          break;
        default:
          fprintf(stderr,"Invalid MAC Control Subtype %x\n", subtype);
          exit(1);
        }
        break;
      case MF_DATA:
        switch(subtype) {
        case MAC_RIL_DATA:
          recvDATA(pktRx_);//newnetif()->decreaserx();
          break;
        default:
          fprintf(stderr, "Invalid MAC Data Subtype %x\n", subtype);
          exit(1);
        }
        break;
      default:
        fprintf(stderr, "Invalid MAC Type %x\n", type);
        exit(1);
  }
  return;
}

//---------------------------- process recv pkt ------------------------//
// process recv data pkt
void RILMac::recvDATA(Packet *p) {
  if (tx_state_ != MAC_IDLE || rx_state_ != MAC_RECV){
    printf("error_z!!! node %d wrong state in line %d\n", index_, __LINE__);
    exit(-1);
  }

  char* mh = (char*)HDR_MAC(p);
  int dst = hdr_dst(mh);
  int src = hdr_src(mh);
  if(dst != BCAST_ADDR) {
    if (running_mac_mode_ != XMAC) inc_sender_cw();
    sendAck(src, sender_cw_, MAC_RIL_ACK);
  }
  else {
    tempCloseRadio(0);
  }
  // notify up layer
  uptarget_->recv(p->copy(), (Handler*) 0);
  Packet::free(p); pktRx_ = 0; rx_state_ = MAC_IDLE;
}

// recv an ack to my data packet
void RILMac::recvACK(Packet *p) {
  double rx_phi, rx_Tr, rx_delay, rx_lifetime;
  int rx_minid, rx_hop;
  if (tx_state_ != MAC_IDLE || rx_state_ != MAC_RECV){
    printf("error_z!!! node %d wrong state in line %d \n", index_, __LINE__);
    exit(-1);
  }
  int src = hdr_src((char*)HDR_MAC(p));
  if(pktTx_ != NULL) {
    if (TsTimer.busy()) TsTimer.stop();
    if (retransmitTimer.busy()) retransmitTimer.stop();
    if (backoffTimer.busy()) backoffTimer.stop();
    if (src != 0) {
        rx_lifetime = hdr_lifetime((char*)HDR_MAC(p));
        rx_minid = hdr_minid((char*)HDR_MAC(p));
        rx_Tr = hdr_tr((char*)HDR_MAC(p))/1000.0;
        rx_delay = hdr_delay((char*)HDR_MAC(p))/1000.0;
        rx_hop = hdr_hop((char*)HDR_MAC(p))+1;
    }
    else {
        rx_lifetime = newnetif()->getLifetime();
        rx_minid = index_;
        rx_Tr = 0.005;
        rx_delay = 0;
        rx_hop = 1;
    }
    rx_phi = hdr_phi((char*)HDR_MAC(p))/1000.0;
    Ts_ = 0;
    updateMyReceiverInfo(src,rx_minid, rx_hop, rx_lifetime, rx_Tr, rx_phi, Ts_, rx_delay);
    Packet::free(pktTx_); pktTx_ = 0; tx_state_ = MAC_IDLE;
    dataRetry_ = 0;
    reset_cw();
    God::instance()->min_upstream_life[index_] = rx_lifetime;
    //God::instance()->min_upstream_id[index_] = rx_minid;
    God::instance()->max_upstream_delay[index_] = rx_delay;
    God::instance()->max_downstream_delay[index_] = tx_delay_;
    God::instance()->my_rxtr[index_] = rx_Tr;
    //God::instance()->current_hop[index_] = rx_hop;
    // rx_minid is area id, and rx_delay is subareaid
    #if 0
    if (src == 0) {
        God::instance()->my_subareaid[index_] = index_;
    }
    else {
        if (rx_minid != God::instance()->my_areaid[index_]) {
            // different area
            God::instance()->my_subareaid[index_] = index_;
        }
        else {
        // same area
            God::instance()->my_subareaid[index_] = hdr_delay((char*)HDR_MAC(p));
        }
    }
    printf("node %d subareaid %d @%f\n",index_,God::instance()->my_subareaid[index_],NOW);
    #endif
  }
  Packet::free(pktRx_); pktRx_ = 0; rx_state_ = MAC_IDLE;
  receiverEntry* e = getMyReceiverInfo(src);
  if ((e->macqueue).size() > 0) {
      pktTx_ = (e->macqueue).front();
      (e->macqueue).pop_front();
  }
  else {
    // no data to current receiver
    for (uint32_t i=0;i<receiverTable.size();i++) {
      if (receiverTable.at(i).macqueue.size()>0) {
          pktTx_ = receiverTable.at(i).macqueue.front();
          receiverTable.at(i).macqueue.pop_front();
          break;
      }
    }
  }
  if (pktTx_) tempToSendData(0, src);
  else tempCloseRadio(0);
}

void RILMac::recvBeaconRRTS(Packet *p) {
  char* mh = (char*)HDR_MAC(p);
  int src = hdr_src(mh);
  unsigned int sendercw = hdr_backoff(mh);
  double backofftime = (Random::random()%(sendercw+1))*config_.backoff_slot;
  bool sending = false;
  Packet::free(pktRx_); pktRx_ = 0; rx_state_ = MAC_IDLE;
  if (running_mac_mode_== XMAC) return;

  if (pktTx_!= NULL) {
    if (hdr_dst((char*)HDR_MAC(pktTx_)) == src) {
        my_cw_ = sendercw;
        sending = true;
        if (TsTimer.busy()) TsTimer.stop();
        if (retransmitTimer.busy()) retransmitTimer.stop();
        tempToSendData(backofftime, src);
    }
  }
  if (sending!=true) {
      int32_t receiverid = -1;
      for (uint32_t i=0;i<receiverTable.size();i++) {
        if (receiverTable.at(i).macqueue.size()>0) {
          if (receiverTable.at(i).id == src) {
            receiverid = i;
            break;
          }
        }
      }
      if (receiverid >= 0) {
        if (pktTx_ != NULL) {
            for (uint32_t i=0;i<receiverTable.size();i++) {
                if (receiverTable.at(i).id == hdr_dst((char*)HDR_MAC(pktTx_))) {
                    receiverTable.at(i).macqueue.push_front(pktTx_);
                    break;
                }
            }
        }
        pktTx_ = receiverTable.at(receiverid).macqueue.front();
        receiverTable.at(receiverid).macqueue.pop_front();
        sending = true;
        if (TsTimer.busy()) TsTimer.stop();
        if (retransmitTimer.busy()) retransmitTimer.stop();
        tempToSendData(backofftime, src);
      }
  }
  if (sending != true) tempCloseRadio(0);
}

// generate a receiver beacon
void RILMac::sendBeacon() {
    if (pktRRTS_!=0) {
        Packet::free(pktRRTS_);
        pktRRTS_=0;
    }
    pktRRTS_ = Packet::alloc();
    hdr_cmn* ch = HDR_CMN(pktRRTS_);
    ch->direction() = hdr_cmn::DOWN;
    ch->size() = config_.header_len;
    ch->uid() = 0;
    ch->ptype() = PT_MAC;
    ch->iface() = -2;
    ch->error() = 0;
    ch->txtime() = (8.0 * ch->size()) / config_.data_rate;
    char *mh = (char*)HDR_MAC(pktRRTS_);
    hdr_mactype(mh, MF_CONTROL);
    hdr_dst(mh, MAC_BROADCAST);
    hdr_src(mh, index_);
    hdr_submactype(mh, MAC_RIL_RRTS);
    double rx_delay;
    if (curRecv != 0) {
      rx_delay = curRecv->delay;
    }
    else {
      rx_delay = 0;
    }
    if (running_mac_mode_!=XMAC) {
        hdr_backoff(mh, sender_cw_);
        hdr_lifetime(mh, (int32_t)(newnetif()->getLifetime()));
        hdr_delay(mh, (int32_t)(rx_delay*1000.0));
    }
    // we didn't do cca checking for beacons in ns2, but should do it in tinyos
    if (tx_state_ == MAC_IDLE && rx_state_ == MAC_IDLE && !backoffTimer.busy()) {
        tx_state_ = MAC_RIMACBEACON;
        downtarget_->recv(pktRRTS_->copy(), this);
        beaconRetry_ = 0;
        senddoneTimer.restart(HDR_CMN(pktRRTS_)->txtime());
    }
    else {
        if (running_mac_mode_ != XMAC) inc_sender_cw();
        tempCloseRadio(0);
        beaconRetry_++;
        if (beaconRetry_ >= 3) {printf("node %d fail to send beacon @ %f\n",index_, NOW);dumpStatus();}
    }
}

// send ack for a data packet
void RILMac::sendAck(int dst, int backoff_cw, MacFrameSubType type) {
  if (pktACK_!=0) {
    Packet::free(pktACK_);
    pktACK_=0;
  }
  pktACK_ = Packet::alloc();
  hdr_cmn* ch = HDR_CMN(pktACK_);
  ch->direction() = hdr_cmn::DOWN;
  ch->size() = config_.ack_len;
  ch->uid() = 0;
  ch->ptype() = PT_MAC;
  ch->iface() = -2;
  ch->error() = 0;
  ch->txtime() = (8.0 * ch->size()) / config_.data_rate;

  char *mh = (char*)HDR_MAC(pktACK_);
  hdr_mactype(mh, MF_CONTROL);
  hdr_submactype(mh, type);
  hdr_dst(mh, dst);
  hdr_src(mh, index_);
  // in ack message, cw, phi and tr should be added
  if (running_mac_mode_!=XMAC) {
      double rx_lifetime,rx_delay;
      int rx_minid, rx_hop;
      hdr_backoff(mh, backoff_cw);
      hdr_tr(mh, (int32_t)(Tr_*1000));
      hdr_phi(mh, (int32_t)(phi_*1000.0));
      if (curRecv != 0) {
        rx_lifetime = curRecv->lifetime;
        rx_delay = curRecv->delay+Tr_;
        rx_minid = curRecv->minid;
        rx_hop = curRecv->hop;
      }
      else {
        rx_lifetime = newnetif()->getLifetime();
        rx_delay = Tr_;
        rx_minid = index_;
        rx_hop = 1;
      }
      //if (newnetif()->getLifetime() < rx_lifetime) {
        rx_lifetime = newnetif()->getLifetime();
        rx_minid = index_;
      //}

      hdr_lifetime(mh, (int32_t)(rx_lifetime));
      //hdr_minid(mh, rx_minid);
      //hdr_delay(mh, (int32_t)(rx_delay*1000.0));
      //hdr_hop(mh, rx_hop);
      hdr_minid(mh, God::instance()->my_areaid[index_]);  // areaid
      hdr_delay(mh, God::instance()->my_subareaid[index_]); // subareaid
      //hdr_hop(mh, God::instance()->my_branchid[index_]); // subareaid
      God::instance()->min_upstream_life[index_] = rx_lifetime;
      //God::instance()->min_upstream_id[index_] = rx_minid;
  }

  if (tx_state_ == MAC_IDLE && rx_state_ == MAC_RECV) {
    tx_state_ = MAC_ACK;
    senddoneTimer.restart(ch->txtime());
    downtarget_->recv(pktACK_->copy(),  (Handler*)0);
  }
  else {
    printf("node %d fail to send ack @ %f\n",index_,NOW);dumpStatus();
    tempCloseRadio(0);
  }
}

// this is the core to send a data pkt
void RILMac::tempToSendData(double backofftime, int trigger) {
  if (pktTx_ == NULL) {
    for (uint32_t i = 0; i<receiverTable.size();i++) {
      if (receiverTable.at(i).macqueue.size()>0) {
        pktTx_ = receiverTable.at(i).macqueue.front();
        receiverTable.at(i).macqueue.pop_front();
        break;
      }
    }
  }
  if (pktTx_ == NULL) {
      tempCloseRadio(0);
      return;
  }

  hdr_cmn* ch = HDR_CMN(pktTx_);
  ch->txtime() = (8.0 * (ch->size() + config_.header_len)) / config_.data_rate;
  if (ch->size() + config_.header_len > 128) {
    ch->txtime() = 8.0*128/config_.data_rate;
  }

  char* mh = (char*)HDR_MAC(pktTx_);
  hdr_mactype(mh, MF_DATA);
  hdr_submactype(mh, MAC_RIL_DATA);
  int dst = hdr_dst(mh);

  newnetif()->turnOnRadio();
  if (dst == trigger || dst == 0) {
    if (backoffTimer.busy()) {
      backoffTimer.stop();
    }
    if (retransmitTimer.busy()) {
      retransmitTimer.stop();
    }
    dataRetry_ = 0;
    reset_cw();
    backoffTimer.restart(backofftime+config_.CCA);//newnetif()->decreasetx();
  }
  else {
    // this is triggered by self-generated or forwarded packet
      if (!TsTimer.busy() && Ts_ > 0) {
        TsTimer.start(Ts_);
      }
  }
}

// TODO: not fully implemented
void RILMac::RetransmitDATA() {
  if(pktTx_ == NULL) {
    printf("node %d invalid pktTx_ in RetransmitDATA\n", index_);
    exit(-1);
  }
  dataRetry_++;
  inc_cw();

  if(dataRetry_ > 4) {
    // give up after the retry threshold
    if (retransmitTimer.busy()) { retransmitTimer.stop();}
    if (backoffTimer.busy()) { backoffTimer.stop();}
    Packet::free(pktTx_); pktTx_ = 0;
    tx_state_ = MAC_IDLE; dataRetry_ = 0;
    printf("node %d fail to send data @ %f\n",index_, NOW);
  }
  else {
    retransmitTimer.restart((Random::random()%(my_cw_+1)) * config_.backoff_slot);
  }
}

// backoff timer handler
void RILMac::backoffHandler() {
  if(rx_state_ == MAC_IDLE && tx_state_ == MAC_IDLE) {
      tx_state_ = MAC_SEND;
      hdr_lifetime((char*)HDR_MAC(pktTx_), (int32_t)(newnetif()->getLifetime()));
      hdr_rate((char*)HDR_MAC(pktTx_), (int32_t)(outgoing_rate_*1000));
      hdr_delay((char*)HDR_MAC(pktTx_), (int32_t)(tx_delay_*1000));
      senddoneTimer.restart(HDR_CMN(pktTx_)->txtime());
      downtarget_->recv(pktTx_->copy(), txHandler_);
  }
  else {
    //channel busy, increase contention window
    double backoff = (Random::random()%(my_cw_+1)) * config_.backoff_slot + config_.ack_len*8.0/config_.data_rate;
    backoffTimer.restart(backoff + config_.CCA);
    inc_cw();
  }
}

// senddone timer handler
void RILMac::senddoneHandler() {
  double minon = 0, rx_Tr = 0;
  // finish sending data
  if (tx_state_ == MAC_SEND) {
    tx_state_ = MAC_IDLE;
    dataRetry_++;
    minon = 2*128*8.0/config_.data_rate+config_.CCA;
    send_scheduledSleep_ = NOW + minon;
    if (curRecv!=0) {
        rx_Tr = curRecv->Tr;
    }
    else rx_Tr = defaulttr_;
    if (running_mac_mode_ != XMAC) {
        if (hdr_arrivaltime((char*)HDR_MAC(pktTx_)) + 2*rx_Tr < NOW) {
            Ts_ = 0;
            if (radioOffTimer.busy()) {
                radioOffTimer.stop();
            }
        }
    }
    // do nothing for xmac, as tstimer is on
  }
  // finish sending ack
  else if (tx_state_ == MAC_ACK) {
    tx_state_ = MAC_IDLE;
    if (pktACK_) {Packet::free(pktACK_); pktACK_ = 0;}
    minon = sender_cw_*config_.backoff_slot+2*128*8.0/config_.data_rate+config_.CCA;
  }
  // finish sending beacon
  else if (tx_state_ == MAC_RIMACBEACON) {
    tx_state_ = MAC_IDLE;
    if (pktRRTS_) {Packet::free(pktRRTS_); pktRRTS_ = 0;}
    minon = sender_cw_*config_.backoff_slot+2*128*8.0/config_.data_rate+config_.CCA;
  }
  else {
    printf("error_zz!!wrong tx state after senddonbackoffTimer.restart(config_.CCA);e %d\n", tx_state_);
    exit(-1);
  }
  tempCloseRadio(minon);
}

void RILMac::tempCloseRadio(double t) {
  double time2sleep = 0;
  double interval = 0;
  if (radioOffTimer.busy()) {
      radioOffTimer.stop();
  }
  if (pktTx_) {
    if (Ts_>0.001 && !TsTimer.busy()) TsTimer.start(Ts_);
    if (running_mac_mode_ == XMAC || Ts_ <= 0.001) {
      newnetif()->turnOnRadio();
      return;
    }
    else {
      // for lbmac, or seesaw, even txmsg exist, radio still can be turned off
    }
  }

  time2sleep = NOW + t; // this is the time you temp to close your radio
  if (time2sleep < recv_scheduledSleep_) time2sleep = recv_scheduledSleep_;
  if (time2sleep < send_scheduledSleep_) time2sleep = send_scheduledSleep_;
  interval = time2sleep - NOW;
  if (interval>0.001) {
    radioOffTimer.start(interval);
  }
  else {
    if(noOngoingTransaction()) {
      if (beaconRetry_<3 && beaconRetry_>0) {
         sendBeacon();
      }
      else newnetif()->turnOffRadio();
    }
    else {printf("node %d cannot turnoff @%f\n",index_,NOW);}
  }
}

// handler of the radio off timer
void RILMac::radioOffHandler() {
  if (pktTx_) {
    if (Ts_>0.001 && !TsTimer.busy()) TsTimer.start(Ts_);
    if (running_mac_mode_ == XMAC || Ts_ <= 0.001) {
      newnetif()->turnOnRadio();
      return;
    }
    else {
      // radio can be turnned off
    }
  }
  if (noOngoingTransaction()) {
      if (beaconRetry_<3 && beaconRetry_>0) {
         sendBeacon();
      }
      else newnetif()->turnOffRadio();
  }
  else {
      // tx/rx state is not idle, backoff or retransmit timer is on
  }
}

// handler of the periodical Tr wakeup timer
void RILMac::wakeupHandler() {
  newnetif()->turnOnRadio();
  if (phi_ == Tr_) recv_scheduledSleep_ = phi_+phi_+NOW;
  else recv_scheduledSleep_ = phi_ + NOW;

  if (radioOffTimer.busy()) {
    radioOffTimer.stop();
  }
  if (running_mac_mode_ != XMAC) {
    sendBeacon();
  }
  else {
    tempCloseRadio(0);
  }
  if (index_ == 0 ) {
    God::instance()->min_upstream_life[index_] = 99999999;
    God::instance()->max_upstream_delay[index_] = 0;
  }
  God::instance()->my_tr[index_] = Tr_;
  God::instance()->my_outrate[index_] = outgoing_rate_;
  if (curRecv != 0) God::instance()->my_rxtr[index_] = curRecv->Tr;
  else God::instance()->my_rxtr[index_] = Tr_;
  wakeupTimer.restart(Tr_ - 0.2 + Random::uniform()*0.2);
}

// handler for data retransmission timer
void RILMac::retransmitHandler() {
  if (pktTx_ != NULL) {
    if (noOngoingTransaction()) {
        backoffTimer.restart(config_.CCA);
    }
  }
  tempCloseRadio(0);
}

// handler for Ts timer
void RILMac::tstimerHandler() {
    if (pktTx_ != NULL) {
        if (noOngoingTransaction()) {
            tempToSendData(0, index_);
        }
        if (Ts_ > 0.001) {
            TsTimer.restart(Ts_);
        }
    }
    else tempCloseRadio(0);
}

// ------------------------ all timers -------------------------//
void RILMacTimer::restart(double time) {
  if (busy_) {
    stop();
  }
  start(time);
}

void RILMacTimer::start(double time) {
  Scheduler &s = Scheduler::instance();

  assert(busy_ == 0);
  busy_ = 1;
  stime = s.clock();
  rtime = time;
  assert(rtime >= 0.0);

  s.schedule(this, &intr, rtime);
}

void RILMacTimer::stop(void) {
  Scheduler &s = Scheduler::instance();

  assert(busy_);
  s.cancel(&intr);
  busy_ = 0;
  stime = rtime = 0.0;
}

void RILMacTimer::forcestop() {
    if (busy_) {
        stop();
    }
}

void RILMacBackoffTimer::handle(Event *e) {
  busy_ = 0;
  stime = rtime = 0.0;
  mac->backoffHandler();
}

void RILMacSenddoneTimer::handle(Event *e){
  busy_ = 0;
  stime = rtime = 0.0;
  mac->senddoneHandler();
}

void RILMacRecvdoneTimer::handle(Event *e){
  busy_ = 0;
  stime = rtime = 0.0;
  mac->recvdoneHandler();
}

void RILMacRetransmitTimer::handle(Event *e){
  busy_ = 0;
  stime = rtime = 0.0;
  mac->retransmitHandler();
}

void RILMacWakeupTimer::handle(Event *e) {
    busy_ = 0;
    stime = rtime = 0.0;
    mac->wakeupHandler();
}

void RILMacTsTimer::handle(Event *e) {
    busy_ = 0;
    stime = rtime = 0.0;
    mac->tstimerHandler();
}

void RILMacRadioOffTimer::handle(Event *e) {
    busy_ = 0;
    stime = rtime = 0.0;
    mac->radioOffHandler();
}


