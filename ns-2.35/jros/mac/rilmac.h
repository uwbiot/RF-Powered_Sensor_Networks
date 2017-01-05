#ifndef ns_rilmac_h
#define ns_rilmac_h


#include <vector>
#include <list>

#include "timer-handler.h"
#include "packet.h"
#include "mac.h"
#include "wireless-chargingphy.h"
#include "god.h"



class RILMac;

enum MacFrameSubType {
  MAC_RIL_DATA = 1,
  MAC_RIL_ACK = 2,
  MAC_RIL_RRTS = 3,
  MAC_RIL_SRTS = 4,
  MAC_RIL_EARLYACK = 5,
};

enum {
  XMAC = 1,
  RIMAC = 2,
  SEESAW = 3,
  LBMAC = 4,
};

struct senderEntry {
  int id;
  bool invalid;
  double lifetime;
  double rate;
  double Tr;
  double phi;
  double delay;
  double lastUpdate;
  senderEntry(int i, double l, double r, double tr, double p, double d=0):id(i),invalid(false),lifetime(l),rate(r),Tr(tr),phi(p),delay(d),lastUpdate(NOW){}
};

struct receiverEntry {
  int id;
  int minid;
  int hop;
  bool invalid;
  double lifetime;
  double Tr;
  double phi;
  double Ts;
  double delay;
  double rate;
  std::list< Packet* > macqueue;
  receiverEntry(int i, int id, double l, double tr, double p, double t=0, double d=0):id(i),minid(id),hop(1),invalid(false),lifetime(l),Tr(tr),phi(p),Ts(t),delay(d){}
};

class RILMac_Config
{
public:
  RILMac_Config() {
    /* Configuration for Standard 802.15.4 MAC */
    data_rate = 250000;
    backoff_slot = 0.00032;
    CCA = 0.000128;
    cw_min = 3;
    cw_max = 127;
    header_len = 16;
    ack_len = 16;
  }
  double data_rate;
  double backoff_slot;
  double CCA;
  unsigned int cw_min;
  unsigned int cw_max;
  int header_len;                 // The length (in bytes) of MAC header
  int ack_len;                    // The length (in bytes) of ACK
};

class RILMacTimer: public Handler {
public:
	RILMacTimer(RILMac* m) : mac(m) {
	  busy_ = 0;
	}
	virtual void handle(Event *e) = 0;
	virtual void restart(double time);
	virtual void start(double time);
	virtual void stop(void);
    virtual void forcestop(void);
	inline int busy(void) { return busy_; }
	inline double expire(void) {
		return ((stime + rtime) - Scheduler::instance().clock());
	}
protected:
	RILMac*     mac;
	int		    busy_;
	Event		  intr;
	double		stime;
	double		rtime;
	double		slottime;
};

// Timer to use for backoff
class RILMacBackoffTimer: public RILMacTimer {
public: RILMacBackoffTimer(RILMac *m) : RILMacTimer(m) {}
	void handle(Event *e);
};

//  Timer to use for finishing sending of packets
class RILMacSenddoneTimer: public RILMacTimer {
public:
	RILMacSenddoneTimer(RILMac *m) : RILMacTimer(m) {}
	void handle(Event *e);
};

// Timer to use for finishing reception of packets
class RILMacRecvdoneTimer: public RILMacTimer {
public:
	RILMacRecvdoneTimer(RILMac *m) : RILMacTimer(m) {}
	void handle(Event *e);
};

// Timer to use for waiting for ack backoff
class RILMacRetransmitTimer: public RILMacTimer {
public:
	RILMacRetransmitTimer(RILMac *m) : RILMacTimer(m) {}
	void handle(Event *e);
};

// Timer to do periodical wakeup
class RILMacWakeupTimer: public RILMacTimer {
public:
  RILMacWakeupTimer(RILMac *m) : RILMacTimer(m){}
	void handle(Event *e);
};

// Timer to
class RILMacTsTimer: public RILMacTimer {
public:
  RILMacTsTimer(RILMac *m) : RILMacTimer(m){}
	void handle(Event *e);
};

// Timer to control radio on/off
class RILMacRadioOffTimer: public RILMacTimer {
public:
	RILMacRadioOffTimer(RILMac *m) : RILMacTimer(m){}
	void            handle(Event *e);
};

class RILMac : public Mac {
public:
	RILMac();
	void recv(Packet *p, Handler *h);
	void send(Packet *p, Handler *h, u_int16_t type);

	void backoffHandler(void);
	void senddoneHandler(void);
	void recvdoneHandler(void);
	void retransmitHandler(void);
	void wakeupHandler(void);
	void radioOffHandler(void);
    void tstimerHandler(void);

	double txtime(Packet *p);

	inline void inc_cw() {
		my_cw_ = (my_cw_ << 1) + 1;
		if(my_cw_ > config_.cw_max) my_cw_ = config_.cw_max;
	}
    inline void inc_sender_cw() {
		sender_cw_ = (sender_cw_ << 1) + 1;
		if(sender_cw_ > config_.cw_max) sender_cw_ = config_.cw_max;
  }
  inline void reset_cw() { my_cw_ = config_.cw_min;}
  inline void reset_sender_cw() { sender_cw_ = config_.cw_min;}

private:
  RILMac_Config   config_;
  std::vector<senderEntry> senderTable;  // store sender's information
  std::vector<receiverEntry> receiverTable;  // store sender's information
  Packet*          pktRx_;         // recv packet
  Packet*          pktTx_;         // send packet (not from mac)
  Packet*          pktRRTS_;       // mac receiver beacon
  Packet*          pktACK_;       // mac ack packet
  MacState         rx_state_;      // incoming state (MAC_RECV or MAC_IDLE)
  MacState         tx_state_;      // outgoing state
  u_int16_t        my_cw_;            // my own contention window
  u_int16_t        sender_cw_;     // sender's cw size decided by receiver
  u_int16_t        dataRetry_;    // retry count for one data packet
  u_int16_t        beaconRetry_;
  int              running_mac_mode_;
  int              defaulttr_;
  double           e2e_delay_req_;
  double           phi_;           // receiver channel checking time
  double           Tr_;            // receiver wakeup interval
  double           Ts_;            // sender retransmit interval
  double           send_scheduledSleep_;
  double           recv_scheduledSleep_;
  double           tx_delay_;
  double           tx_lifetime_;
  double           incoming_rate_;
  double           outgoing_rate_;
  double           lastUpdate_;
  double           last_increase_time_;
  double           last_decrease_time_;
  uint32_t         lastUpdateCount_;
  uint32_t         total_outgoing_;
  uint32_t         total_incoming_;
  receiverEntry*  curRecv;

  Handler* 	txHandler_;
  RILMacBackoffTimer backoffTimer;      // CCA and contension backoff timer
  RILMacSenddoneTimer senddoneTimer;      // pkt transmission timer
  RILMacRecvdoneTimer recvdoneTimer;      // pkt receiption timer
  RILMacRetransmitTimer  retransmitTimer;       // ack waiting and data retranmission timer
  RILMacWakeupTimer wakeupTimer;    // control when to get awake
  RILMacRadioOffTimer radioOffTimer;  // turn off radio timer
  RILMacTsTimer TsTimer; // sender request-to-send retry timer

  inline WirelessChargingPhy* newnetif(){return dynamic_cast<WirelessChargingPhy*>(netif_);}

  u_int16_t hdr_mactype(char* hdr, uint16_t type=0);
  u_int16_t hdr_submactype(char* hdr, uint16_t type=0);
  u_int16_t hdr_phi(char *hdr, int32_t value=-1);
  u_int16_t hdr_tr(char* hdr, int32_t value=-1);
  u_int16_t hdr_backoff(char* hdr, int32_t value=-1);
  u_int16_t hdr_delay(char *hdr, int32_t value = -1);
  u_int16_t hdr_rate(char* hdr, int32_t value=-1);
  u_int16_t hdr_lifetime(char* hdr, int32_t value=-1);
  u_int16_t hdr_minid(char* hdr, int32_t value=-1);
  u_int16_t hdr_hop(char* hdr, int32_t value=-1);
  double hdr_arrivaltime(char* hdr, double value=-1);

  bool noOngoingTransaction();
  void dumpStatus();


  double updateMySenderlifetime(int id, double lifetime, double rate, double delay);
  void updateMyReceiverInfo(int id, int minid, int hop, double lifetime, double tr, double phi, double ts, double delay);

  receiverEntry* getMyReceiverInfo(int id);
  void tempToSendData(double backofftime, int trigger);

  void sendBeacon();
  void sendAck(int, int, MacFrameSubType);
  void RetransmitDATA();

  void recvDATA(Packet* p);
  void recvACK(Packet* p);
  void recvBeaconRRTS(Packet* p);

  void tempCloseRadio(double t);
};

#endif







