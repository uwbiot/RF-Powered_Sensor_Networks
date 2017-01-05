/* -*-	Mode:C++; c-basic-offset:8; tab-width:8; indent-tabs-mode:t -*-
 *
 * Copyright (c) 1997 Regents of the University of California.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed by the Computer Systems
 *	Engineering Group at Lawrence Berkeley Laboratory.
 * 4. Neither the name of the University nor of the Laboratory may be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $Header: /cvsroot/nsnam/ns-2/mac/mac-802_11.h,v 1.29 2008/12/13 23:22:58 tom_henderson Exp $
 *
 * Ported from CMU/Monarch's code, nov'98 -Padma.
 * wireless-mac-802_11.h
 *
 * Improved by Yanjun at the Monarch group of Rice University
 * for asynchronous duty cycle MAC protocols in wireless 
 * sensor networks, 2009
 */

#ifndef ns_mac_80211_h
#define ns_mac_80211_h

// Added by Sushmita to support event tracing (singal@nunki.usc.edu)
#include "address.h"
#include "ip.h"

#include "mac-timers.h"
#include "marshall.h"
#include <math.h>
#include <stddef.h>
#include <list>
//#include <yjrouteflip.h>
#include <dsr-priqueue.h>

#undef YJ_BMAC_SUPPORT
#define YJ_QUEUED_PKT_WAITER

#define START_MEASUREMENT_THRESHOLD 10

class EventTrace;

#define GET_ETHER_TYPE(x)		GET2BYTE((x))
#define SET_ETHER_TYPE(x,y)            {u_int16_t t = (y); STORE2BYTE(x,&t);}

/* ======================================================================
   Frame Formats
   ====================================================================== */

#define	MAC_ProtocolVersion	0x00

#define MAC_Type_Management	0x00
#define MAC_Type_Control	0x01
#define MAC_Type_Data		0x02
#define MAC_Type_Reserved	0x03

#define MAC_Subtype_RTS		0x0B
#define MAC_Subtype_CTS		0x0C
#define MAC_Subtype_ACK		0x0D
#define MAC_Subtype_Data	0x00

struct frame_control {
	u_char		fc_subtype		: 4;
	u_char		fc_type			: 2;
	u_char		fc_protocol_version	: 2;

	u_char		fc_order		: 1;
	u_char		fc_wep			: 1;
	u_char		fc_more_data		: 1;
	u_char		fc_pwr_mgt		: 1;
	u_char		fc_retry		: 1;
	u_char		fc_more_frag		: 1;
	u_char		fc_from_ds		: 1;
	u_char		fc_to_ds		: 1;
};

struct rts_frame {
	struct frame_control	rf_fc;
	u_int16_t		rf_duration;
	u_char			rf_ra[ETHER_ADDR_LEN];
	u_char			rf_ta[ETHER_ADDR_LEN];
	u_char			rf_fcs[ETHER_FCS_LEN];
};

struct cts_frame {
	struct frame_control	cf_fc;
	u_int16_t		cf_duration;
	u_char			cf_ra[ETHER_ADDR_LEN];
	u_char			cf_fcs[ETHER_FCS_LEN];
};

struct ack_frame {
	struct frame_control	af_fc;
	u_int16_t		af_duration;
	u_char			af_ra[ETHER_ADDR_LEN];
	// as this ACK also serves as a BEACON
	u_char			af_ta[ETHER_ADDR_LEN];
	u_int16_t		cw; // backoff window
	u_int16_t		intended_receiver;
	u_char			af_fcs[ETHER_FCS_LEN];
};

// XXX This header does not have its header access function because it shares
// the same header space with hdr_mac.
struct hdr_mac802_11 {
	struct frame_control	dh_fc;
	u_int16_t		dh_duration;
	u_char                  dh_ra[ETHER_ADDR_LEN];
        u_char                  dh_ta[ETHER_ADDR_LEN];
        u_char                  dh_3a[ETHER_ADDR_LEN];
	u_int16_t		dh_scontrol;
	u_char			dh_body[0]; // XXX Non-ANSI
};


/* ======================================================================
   Definitions
   ====================================================================== */

/* Must account for propagation delays added by the channel model when
 * calculating tx timeouts (as set in tcl/lan/ns-mac.tcl).
 *   -- Gavin Holland, March 2002
 */
#define DSSS_MaxPropagationDelay        0.000002        // 2us   XXXX

class PHY_MIB {
public:
	PHY_MIB(Mac802_11 *parent);

	inline u_int32_t getCWMin() { return(CWMin); }
        inline u_int32_t getCWMax() { return(CWMax); }
	inline double getSlotTime() { return(SlotTime); }
	inline double getSIFS() { return(SIFSTime); }
	inline double getPIFS() { return(SIFSTime + SlotTime); }
	inline double getDIFS() { 
#ifdef YJ_802_15_4_PARAM
		// don't need difs based on figure 61
		return 0.;
#else
		return(SIFSTime + 2 * SlotTime); 
#endif
	}
	inline double getEIFS() {
#ifdef YJ_802_15_4_PARAM
		// don't need eifs based on figure 61
		return 0.;
#else
		// see (802.11-1999, 9.2.10)
		return(SIFSTime + getDIFS()
                       + (8 *  getACKlen())/PLCPDataRate);
#endif
	}
	inline u_int32_t getPreambleLength() { return(PreambleLength); }
	inline double getPLCPDataRate() { return(PLCPDataRate); }
	
	inline u_int32_t getPLCPhdrLen() {
		return((PreambleLength + PLCPHeaderLength) >> 3);
	}

	inline u_int32_t getHdrLen11() {
#ifdef YJ_802_15_4_PARAM
		return(getPLCPhdrLen() + 7); // based on TEP 111 for CC2420 radio
#else
		return(getPLCPhdrLen() + sizeof(struct hdr_mac802_11)
                       + ETHER_FCS_LEN);
#endif
	}
	
	inline u_int32_t getRTSlen() {
#ifdef YJ_802_15_4_PARAM
		return 16;
#else
		return(getPLCPhdrLen() + sizeof(struct rts_frame));
#endif
	}
	
	inline u_int32_t getCTSlen() {
#ifdef YJ_802_15_4_PARAM
		return 16;
#else
		return(getPLCPhdrLen() + sizeof(struct cts_frame));
#endif
	}
	
	inline u_int32_t getACKlen() {
#ifdef YJ_802_15_4_PARAM
		return(getPLCPhdrLen() + 5); // 11 bytes; based on cc2420 datasheet & 802.15.4 standard
#else
		return(getPLCPhdrLen() + sizeof(struct ack_frame));
#endif
	}

 private:




	u_int32_t	CWMin;
	u_int32_t	CWMax;
	double		SlotTime;
	double		SIFSTime;
	u_int32_t	PreambleLength;
	u_int32_t	PLCPHeaderLength;
	double		PLCPDataRate;
};


/*
 * IEEE 802.11 Spec, section 11.4.4.2
 *      - default values for the MAC Attributes
 */
#define MAC_FragmentationThreshold	2346		// bytes
#define MAC_MaxTransmitMSDULifetime	512		// time units
#define MAC_MaxReceiveLifetime		512		// time units

class MAC_MIB {
public:

	MAC_MIB(Mac802_11 *parent);

private:
	u_int32_t	RTSThreshold;
	u_int32_t	ShortRetryLimit;
	u_int32_t	LongRetryLimit;
public:
	u_int32_t	FailedCount;	
	u_int32_t	RTSFailureCount;
	u_int32_t	ACKFailureCount;
 public:
       inline u_int32_t getRTSThreshold() { return(RTSThreshold);}
       inline u_int32_t getShortRetryLimit() { return(ShortRetryLimit);}
       inline u_int32_t getLongRetryLimit() { return(LongRetryLimit);}
};


/* ======================================================================
   The following destination class is used for duplicate detection.
   ====================================================================== */
class Host {
public:
	LIST_ENTRY(Host) link;
	u_int32_t	index;
	u_int32_t	seqno;
};

#ifndef yj_neighbor
#define yj_neighbor
#define yjclock (Scheduler::instance().clock())

#define MAC_Subtype_RRTS	0x0E

struct rrts_frame {
	struct frame_control	rf_fc;
	u_int16_t		rf_duration;
	u_char			rf_ra[ETHER_ADDR_LEN];
	u_char			rf_ta[ETHER_ADDR_LEN];
	u_int16_t		cw; // backoff window
	u_int16_t		intended_receiver;
	u_int16_t		on_demand;
	u_char			rf_fcs[ETHER_FCS_LEN];
};

class MACHelloTimer: public MacTimer {
public:
	MACHelloTimer(Mac802_11 *m) : MacTimer(m) {}

	void	start(int, double);
	void	handle(Event *e);
};

class Cell{
public:
	Cell(){
		time = 0.0;
		pkt = NULL;
		next = NULL;
	}
	~Cell(){
		if(pkt != NULL)
			Packet::free(pkt);
		if( next != NULL ){
			delete next;
		}
	}
	double time;
	Packet *pkt;
	Cell *next;
}; // class Cell

class myneighbor_list_item {
public:
//enum link_stat {OK, SUSPICIOUS, BROKEN, UNKNOWN};
// extended contents
myneighbor_list_item(int macindex, u_int32_t initCW = 0);
~myneighbor_list_item();
int macindex_; // my mac id
u_int32_t id;      		// neighbor id
myneighbor_list_item *next; 	// pointer to next item
double transmission_bound_time_;
bool if_collision_;
u_int32_t cw_;
};

struct myneighborlist{
int neighbor_cnt_;   // how many neighbors in this list
myneighbor_list_item *head; 
} ;
#endif 


class SignalFinishTime{
public:
	SignalFinishTime(double t, double s){
		finish_time_ = t;
		strength_ = s;
		next_ = NULL;
	}
	double finish_time_;
	double strength_;
	struct SignalFinishTime *next_;
};

class SINRMonitor : public MacTimer{
public:
	SINRMonitor(Mac802_11 *m);
	virtual void	handle(Event *e);
	Mac802_11 *mac_;
	struct SignalFinishTime * head_;
	int count_;
	double totalpw_;
	double time_of_last_bit_;

	void cleanup(double curtime);
	double getTotal(double curtime);
	bool channelClear(double curtime, double CSThresh);

	void add(double transmissiontime, Packet *p, double curtime);
};

class IdleGuardTimer : public MacTimer {
public:
	IdleGuardTimer(Mac802_11 *m) : MacTimer(m) {}

	void	start(double);
	void	handle(Event *e);
};

#ifdef YJ_QUEUED_PKT_WAITER
class QedPktWaitTimer: public MacTimer {
public:
	QedPktWaitTimer(Mac802_11 *m) : MacTimer(m) {}

	void	start(double);
	void	handle(Event *e);
};
#endif // YJ_QUEUED_PKT_WAITER

/* ======================================================================
   The actual 802.11 MAC class.
   ====================================================================== */
class Mac802_11 : public Mac {
	friend class DeferTimer;


	friend class BackoffTimer;
	friend class IFTimer;
	friend class NavTimer;
	friend class RxTimer;
	friend class TxTimer;
public:
	Mac802_11();
	void		recv(Packet *p, Handler *h);
	inline int	hdr_dst(char* hdr, int dst = -2);
	inline int	hdr_src(char* hdr, int src = -2);
	inline int	hdr_type(char* hdr, u_int16_t type = 0);
	
	inline int bss_id() { return bss_id_; }
	
	// Added by Sushmita to support event tracing
        void trace_event(char *, Packet *);
        EventTrace *et_;

protected:
	virtual void	backoffHandler(void);
	virtual void	deferHandler(void);
	virtual void	navHandler(void);
	virtual void	recvHandler(void);
	virtual void	sendHandler(void);
	virtual void	txHandler(void);

private:
	int		command(int argc, const char*const* argv);

	/*
	 * Called by the timers.
	 */
	void		recv_timer(void);
	void		send_timer(void);
	int		check_pktCTRL();
	int		check_pktTx(bool ignoremedium = false);

	/*
	 * Packet Transmission Functions.
	 */
	void	send(Packet *p, Handler *h);
	void	sendACK(int dst);
	void	sendDATA(Packet *p);
	void	RetransmitDATA();

	/*
	 * Packet Reception Functions.
	 */
	void	recvACK(Packet *p);
	void	recvDATA(Packet *p);

	void		capture(Packet *p);
	void		collision(Packet *p);
	void		discard(Packet *p, const char* why);
	void		rx_resume(void);
	void		tx_resume(void);

	inline int	is_idle(void);

	/*
	 * Debugging Functions.
	 */
	void		trace_pkt(Packet *p);
	void		dump(char* fname);

	inline int initialized() {	
		return (cache_ && logtarget_
                        && Mac::initialized());
	}

	inline void mac_log(Packet *p) {
                logtarget_->recv(p, (Handler*) 0);
        }

	double txtime(Packet *p);
	double txtime(double psz, double drt);
	double txtime(int bytes) { /* clobber inherited txtime() */ abort(); return 0;}

	inline void transmit(Packet *p, double timeout);
	inline void checkBackoffTimer(void);
	inline void postBackoff(int pri);
	void setRxState(MacState newState, bool forced = false);
	void setTxState(MacState newState, bool forced = false);
	void forceRxState(MacState newState);
	void forceTxState(MacState newState);


	inline void inc_cw() {
		cw_ = (cw_ << 1) + 1;
		if(cw_ > phymib_.getCWMax())
			cw_ = phymib_.getCWMax();
	}
	inline void rst_cw() { cw_ = phymib_.getCWMin(); }

	inline double sec(double t) { return(t *= 1.0e-6); }
	inline u_int16_t usec(double t) {
		u_int16_t us = (u_int16_t)floor((t *= 1e6) + 0.5);
		return us;
	}

protected:
	PHY_MIB         phymib_;
        MAC_MIB         macmib_;

       /* the macaddr of my AP in BSS mode; for IBSS mode
        * this is set to a reserved value IBSS_ID - the
        * MAC_BROADCAST reserved value can be used for this
        * purpose
        */
       int     bss_id_;
       enum    {IBSS_ID=MAC_BROADCAST};


private:
	double		basicRate_;
 	double		dataRate_;
	
	/*
	 * Mac Timers
	 */
	IFTimer		mhIF_;		// interface timer
	RxTimer		mhRecv_;		// incoming packets
	TxTimer		mhSend_;		// outgoing packets

	DeferTimer	mhDefer_;	// defer timer
	BackoffTimer	mhBackoff_;	// backoff timer

	/* ============================================================
	   Internal MAC State
	   ============================================================ */
	double		nav_;		// Network Allocation Vector

	MacState	rx_state_;	// incoming state (MAC_RECV or MAC_IDLE)
	MacState	tx_state_;	// outgoint state
	int		tx_active_;	// transmitter is ACTIVE

	Packet          *eotPacket_;    // copy for eot callback

	Packet		*pktCTRL_;	// outgoing ack packet

	u_int32_t	cw_;		// Contention Window
	u_int32_t	ssrc_;		// STA Short Retry Count
	u_int32_t	slrc_;		// STA Long Retry Count

	int		min_frame_len_;

//	NsObject*	logtarget_;
	NsObject*       EOTtarget_;     // given a copy of packet at TX end




	/* ============================================================
	   Duplicate Detection state
	   ============================================================ */
	u_int16_t	sta_seqno_;	// next seqno that I'll use
	int		cache_node_count_;
	Host		*cache_;

public:
	CMUPriQueue *ifq_; // points to IFQ
	bool attemptToSleep(bool forced = false);
	bool wakeup();
	bool is_transmission_attempt_allowed(bool is_boxxmac_preamble = false);
	double duty_cycle_length_;
	bool if_want_sleep_;
	double last_radio_state_update_time_;
	bool duty_cycle_stat_started_; // whether to collect statistical infomation on duty cycle
	virtual void handleWakeup();
	virtual void idleGuardTimerHandler();
	// only for rdmac
	virtual void discardTimerHandler(){};
#ifdef YJ_QUEUED_PKT_WAITER
	QedPktWaitTimer	mhQedPktWaiter_;
	virtual void QedPktWaitTimerHandler();
	double dwell_time_;
#endif
	IdleGuardTimer mhIdle_;
	bool is_waiting_for_pkt_;
	virtual void decideWhetherToSleep();

#ifdef YJ_XMAC_EXTENSION
	// specific to XMAC
	void setupXMACPreamble(int dst);
	void recvPreamble(Packet *p);
#endif

	SINRMonitor sinrMonitor;
	virtual void sinrMonitorHandler();
	double CSThresh_;

	// if the given pkt's PLCP header is being received 
	bool isPLCPHdrDone(Packet *p);
	double neighbor_timeout;
	Trace*	logtarget_;
	double hello_upper_bound;
	void recv_beacon(u_int32_t nodemac);

	struct myneighborlist myneighbor_list;
	void start_neighbourdetect();
	void start_hello();
	myneighbor_list_item* getNeighbor(u_int32_t dmac);
	MACHelloTimer* hellotimer_;
	void direct_trace(char* fmt, ...);


	//flags
	static bool beacon_flag;

};



#endif /* __mac_80211_h__ */

