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
 * Ported from CMU/Monarch's code, nov'98 -Padma.
 * wireless-mac-802_11.h
 *
 * Improved by Yanjun at the Monarch group of Rice University
 * for asynchronous duty cycle MAC protocols in wireless 
 * sensor networks, 2009
 */
 
#include "delay.h"
#include "connector.h"
#include "packet.h"
#include "random.h"
#include "mobilenode.h"

#include "arp.h"
#include "ll.h"
#include "mac.h"
#include "mac-timers.h"
#include "wireless-phy.h"
#include "mac-802_11.h"
#include "cmu-trace.h"

// Added by Sushmita to support event tracing
#include "agent.h"
#include "basetrace.h"

#include "dsr/hdr_sr.h"
#include <iostream>
#include <set>
using namespace std;
#define TIME_NOW (Scheduler::instance().clock())


bool Mac802_11::beacon_flag = true;

inline void
Mac802_11::checkBackoffTimer()
{
	if( is_idle() && mhBackoff_.paused())
		mhBackoff_.resume(phymib_.getDIFS());
	if(! is_idle() && mhBackoff_.busy() && ! mhBackoff_.paused())
		mhBackoff_.pause();
}

inline void
Mac802_11::transmit(Packet *p, double timeout)
{
	tx_active_ = 1;
	
	if (EOTtarget_) {
		assert (eotPacket_ == NULL);
		eotPacket_ = p->copy();
	}

	/*
	 * If I'm transmitting without doing CS, such as when
	 * sending an ACK, any incoming packet will be "missed"
	 * and hence, must be discarded.
	 */
	if(rx_state_ != MAC_IDLE) {
		assert(pktRx_);
		if( pktRx_ ){
			struct hdr_cmn *ch = HDR_CMN(pktRx_);
			ch->error() = 1;        /* force packet discard */
		}
	}

	/*
	 * pass the packet on the "interface" which will in turn
	 * place the packet on the channel.
	 *
	 * NOTE: a handler is passed along so that the Network
	 *       Interface can distinguish between incoming and
	 *       outgoing packets.
	 */
	downtarget_->recv(p->copy(), this);	
	mhSend_.start(timeout);
	mhIF_.start(txtime(p));
}

void Mac802_11::forceRxState(MacState newState){
	rx_state_ = newState;
}
void Mac802_11::forceTxState(MacState newState){
	tx_state_ = newState;
}

void Mac802_11::setRxState(MacState newState, bool force)
{
	// ignore any state update if it's sleep state now
	if( rx_state_ == MAC_SLEEP ){
		if( newState != MAC_IDLE ){
			fprintf(stderr, "### Warning: %d ignore newstate %d in setRxState because node is sleeping\n at %f\n", index_, newState, TIME_NOW);
		}

		return;
	}
	if( force ){
		rx_state_ = newState;
		return;
	}


	rx_state_ = newState;
	if( newState == MAC_IDLE && if_want_sleep_ && tx_state_ == MAC_IDLE && attemptToSleep())
		return;
	checkBackoffTimer();
}

void Mac802_11::setTxState(MacState newState, bool forced)
{
	// ignore any state update if it's sleep state now
	if( tx_state_ == MAC_SLEEP ) return;

	tx_state_ = newState;
	if( forced ){
		return;
	}
	if( newState == MAC_IDLE && if_want_sleep_ && rx_state_ == MAC_IDLE && attemptToSleep())
		return;
	checkBackoffTimer();
}


/* ======================================================================
   TCL Hooks for the simulator
   ====================================================================== */
static class Mac802_11Class : public TclClass {
public:
	Mac802_11Class() : TclClass("Mac/802_11") {}
	TclObject* create(int, const char*const*) {
	return (new Mac802_11());

}
} class_mac802_11;


/* ======================================================================
   Mac  and Phy MIB Class Functions
   ====================================================================== */

PHY_MIB::PHY_MIB(Mac802_11 *parent)
{
	/*
	 * Bind the phy mib objects.  Note that these will be bound
	 * to Mac/802_11 variables
	 */

	parent->bind("CWMin_", &CWMin);
	parent->bind("CWMax_", &CWMax);
	parent->bind("SlotTime_", &SlotTime);
	parent->bind("SIFS_", &SIFSTime);
	parent->bind("PreambleLength_", &PreambleLength);
	parent->bind("PLCPHeaderLength_", &PLCPHeaderLength);
	parent->bind_bw("PLCPDataRate_", &PLCPDataRate);
}

MAC_MIB::MAC_MIB(Mac802_11 *parent)
{
	/*
	 * Bind the phy mib objects.  Note that these will be bound
	 * to Mac/802_11 variables
	 */
	
	parent->bind("ShortRetryLimit_", &ShortRetryLimit);
	parent->bind("LongRetryLimit_", &LongRetryLimit);
}

/* ======================================================================
   Mac Class Functions
   ====================================================================== */
Mac802_11::Mac802_11() : 
	Mac(), phymib_(this), macmib_(this), mhIF_(this), 
	mhRecv_(this), mhSend_(this), 
	mhDefer_(this), mhBackoff_(this)
#ifdef YJ_QUEUED_PKT_WAITER
	, mhQedPktWaiter_(this)
#endif
	, mhIdle_(this)
	, sinrMonitor(this)
{
	
	nav_ = 0.0;
	tx_state_ = rx_state_ = MAC_IDLE;
	tx_active_ = 0;
	eotPacket_ = NULL;
	pktCTRL_ = 0;		
	cw_ = phymib_.getCWMin();
	ssrc_ = slrc_ = 0;
	// Added by Sushmita
        et_ = new EventTrace();
	
	sta_seqno_ = 1;
	cache_ = 0;
	cache_node_count_ = 0;
	
	// chk if basic/data rates are set
	// otherwise use bandwidth_ as default;
	
	Tcl& tcl = Tcl::instance();
	tcl.evalf("Mac/802_11 set basicRate_");
	if (strcmp(tcl.result(), "0") != 0) 
		bind_bw("basicRate_", &basicRate_);
	else
		basicRate_ = bandwidth_;

	tcl.evalf("Mac/802_11 set dataRate_");
	if (strcmp(tcl.result(), "0") != 0) 
		bind_bw("dataRate_", &dataRate_);
	else
		dataRate_ = bandwidth_;

	EOTtarget_ = 0;
	bss_id_ = IBSS_ID;

	bind("CSThresh_", &CSThresh_);

	bind("DutyCycleLength_", &duty_cycle_length_);
	tx_state_ = rx_state_ = MAC_SLEEP;
	last_radio_state_update_time_ = 0.;
	duty_cycle_stat_started_ = false;

}


int
Mac802_11::command(int argc, const char*const* argv)
{
	if (argc == 2) {
		if (strcmp(argv[1], "start-neighbor") == 0) {
			if( index_ ){
				if(beacon_flag){
					start_neighbourdetect();
					start_hello();
				}
			}

			return TCL_OK;
		} else if (strcmp(argv[1], "updateDutyCycle") == 0) {
			// handle the corner where a node never sleep/wakeup after START_MEASUREMENT_THRESHOLD
			if( duty_cycle_stat_started_ == false ){
				last_radio_state_update_time_ = START_MEASUREMENT_THRESHOLD;
			}
			// dutycycle counter
			if( tx_state_ != MAC_SLEEP ){
				God::instance()->total_active_time_ += TIME_NOW - last_radio_state_update_time_;
				God::instance()->duty_cycle_active_[index_] += TIME_NOW - last_radio_state_update_time_;
			} else {
				God::instance()->total_sleep_time_ += TIME_NOW - last_radio_state_update_time_;
				God::instance()->duty_cycle_sleep_[index_] += TIME_NOW - last_radio_state_update_time_;
			}

			last_radio_state_update_time_ = TIME_NOW;

			return TCL_OK;
		}
	} else if (argc == 3) {
		if (strcmp(argv[1], "eot-target") == 0) {
			EOTtarget_ = (NsObject*) TclObject::lookup(argv[2]);
			if (EOTtarget_ == 0)
				return TCL_ERROR;
			return TCL_OK;
		} else if (strcmp(argv[1], "bss_id") == 0) {
			bss_id_ = atoi(argv[2]);
			return TCL_OK;
		} else if (strcmp(argv[1], "log-target") == 0) { 
			logtarget_ = (Trace*) TclObject::lookup(argv[2]);
			//logtarget_ = (NsObject*) TclObject::lookup(argv[2]);
			if(logtarget_ == 0)
				return TCL_ERROR;
			return TCL_OK;
		} else if(strcmp(argv[1], "nodes") == 0) {
			if(cache_) return TCL_ERROR;
			cache_node_count_ = atoi(argv[2]);
			cache_ = new Host[cache_node_count_ + 1];
			assert(cache_);
			bzero(cache_, sizeof(Host) * (cache_node_count_+1 ));
			return TCL_OK;
		} else if(strcmp(argv[1], "eventtrace") == 0) {
			// command added to support event tracing by Sushmita
                        et_ = (EventTrace *)TclObject::lookup(argv[2]);
                        return (TCL_OK);
                }
	}
	return Mac::command(argc, argv);
}

// Added by Sushmita to support event tracing
void Mac802_11::trace_event(char *eventtype, Packet *p) 
{
        if (et_ == NULL) return;
        char *wrk = et_->buffer();
        char *nwrk = et_->nbuffer();
	
        //char *src_nodeaddr =
	//       Address::instance().print_nodeaddr(iph->saddr());
        //char *dst_nodeaddr =
        //      Address::instance().print_nodeaddr(iph->daddr());
	
        struct hdr_mac802_11* dh = HDR_MAC802_11(p);
	
	if(wrk != 0) {
		sprintf(wrk, "E -t "TIME_FORMAT" %s %2x ",
			et_->round(Scheduler::instance().clock()),
                        eventtype,
                        ETHER_ADDR(dh->dh_ta)
                        );
        }
        if(nwrk != 0) {
                sprintf(nwrk, "E -t "TIME_FORMAT" %s %2x ",
                        et_->round(Scheduler::instance().clock()),
                        eventtype,
                        ETHER_ADDR(dh->dh_ta)
                        );
        }
        et_->dump();
}

/* ======================================================================
   Debugging Routines
   ====================================================================== */
void
Mac802_11::trace_pkt(Packet *p) 
{
	struct hdr_cmn *ch = HDR_CMN(p);
	struct hdr_mac802_11* dh = HDR_MAC802_11(p);
	u_int16_t *t = (u_int16_t*) &dh->dh_fc;

	fprintf(stderr, "\t[ %2x %2x %2x %2x ] %x %s %d\n",
		*t, dh->dh_duration,
		 ETHER_ADDR(dh->dh_ra), ETHER_ADDR(dh->dh_ta),
		index_, packet_info.name(ch->ptype()), ch->size());
}

void
Mac802_11::dump(char *fname)
{
	fprintf(stderr,
		"\n%s --- (INDEX: %d, time: %2.9f)\n",
		fname, index_, Scheduler::instance().clock());

	fprintf(stderr,
		"\ttx_state_: %x, rx_state_: %x, nav: %2.9f, idle: %d\n",
		tx_state_, rx_state_, nav_, is_idle());

	fprintf(stderr,
		"\tpktTx_: %lx, pktRx_: %lx, pktCTRL_: %lx, callback: %lx\n",
		(long) pktTx_, (long) pktRx_,
		(long) pktCTRL_, (long) callback_);

	fprintf(stderr,
		"\tDefer: %d, Backoff: %d (%d), Recv: %d, Timer: %d\n",
		mhDefer_.busy(), mhBackoff_.busy(), mhBackoff_.paused(),
		mhRecv_.busy(), mhSend_.busy());
	fprintf(stderr,
		"\tBackoff Expire: %f\n",
		mhBackoff_.expire());
}


/* ======================================================================
   Packet Headers Routines
   ====================================================================== */
inline int
Mac802_11::hdr_dst(char* hdr, int dst )
{
	struct hdr_mac802_11 *dh = (struct hdr_mac802_11*) hdr;
	
       if (dst > -2) {
               if ((bss_id() == ((int)IBSS_ID)) || (addr() == bss_id())) {
                       /* if I'm AP (2nd condition above!), the dh_3a
                        * is already set by the MAC whilst fwding; if
                        * locally originated pkt, it might make sense
                        * to set the dh_3a to myself here! don't know
                        * how to distinguish between the two here - and
                        * the info is not critical to the dst station
                        * anyway!
                        */
                       STORE4BYTE(&dst, (dh->dh_ra));
               } else {
                       /* in BSS mode, the AP forwards everything;
                        * therefore, the real dest goes in the 3rd
                        * address, and the AP address goes in the
                        * destination address
                        */
                       STORE4BYTE(&bss_id_, (dh->dh_ra));
                       STORE4BYTE(&dst, (dh->dh_3a));
               }
       }


       return (u_int32_t)ETHER_ADDR(dh->dh_ra);
}

inline int 
Mac802_11::hdr_src(char* hdr, int src )
{
	struct hdr_mac802_11 *dh = (struct hdr_mac802_11*) hdr;
        if(src > -2)
               STORE4BYTE(&src, (dh->dh_ta));
        return ETHER_ADDR(dh->dh_ta);
}

inline int 
Mac802_11::hdr_type(char* hdr, u_int16_t type)
{
	struct hdr_mac802_11 *dh = (struct hdr_mac802_11*) hdr;
	if(type)
		STORE2BYTE(&type,(dh->dh_body));
	return GET2BYTE(dh->dh_body);
}


/* ======================================================================
   Misc Routines
   ====================================================================== */
inline int
Mac802_11::is_idle()
{
	if(rx_state_ != MAC_IDLE)
		return 0;
	if(tx_state_ != MAC_IDLE)
		return 0;
	if(nav_ > Scheduler::instance().clock())
		return 0;
	
	return 1;
}


void
Mac802_11::discard(Packet *p, const char* why)
{
	hdr_mac802_11* mh = HDR_MAC802_11(p);
	hdr_cmn *ch = HDR_CMN(p);

	/* if the rcvd pkt contains errors, a real MAC layer couldn't
	   necessarily read any data from it, so we just toss it now */
	if(ch->error() != 0) {
		Packet::free(p);
		return;
	}

	switch(mh->dh_fc.fc_type) {
	case MAC_Type_Management:
		drop(p, why);
		return;
	case MAC_Type_Control:
		switch(mh->dh_fc.fc_subtype) {
		case MAC_Subtype_ACK:
		default:
			fprintf(stderr, "discard: invalid MAC Control subtype at node %d at %f\n", index_, TIME_NOW);
			exit(1);
		}
		break;
	case MAC_Type_Data:
		switch(mh->dh_fc.fc_subtype) {
		case MAC_Subtype_Data:
			if((u_int32_t)ETHER_ADDR(mh->dh_ra) == \
                           (u_int32_t)index_ ||
                          (u_int32_t)ETHER_ADDR(mh->dh_ta) == \
                           (u_int32_t)index_ ||
                          (u_int32_t)ETHER_ADDR(mh->dh_ra) == MAC_BROADCAST) {
                                drop(p,why);
                                return;
			}
			break;
		default:
			fprintf(stderr, "invalid MAC Data subtype\n");
			exit(1);
		}
		break;
	default:
		fprintf(stderr, "invalid MAC type (%x)\n", mh->dh_fc.fc_type);
		trace_pkt(p);
		exit(1);
	}
	Packet::free(p);
}

void
Mac802_11::capture(Packet *p)
{
	/*
	 * Update the NAV so that this does not screw
	 * up carrier sense.
	 */	

	Packet::free(p);
}


bool Mac802_11::isPLCPHdrDone(Packet *p){
	struct hdr_cmn *ch = HDR_CMN(p);
	if( ch->arrival_time_ + 8.*phymib_.getPLCPhdrLen()/ phymib_.getPLCPDataRate() >= NOW ){
		return true;
	} else {
		return false;
	}
}


void
Mac802_11::collision(Packet *p)
{
	switch(rx_state_) {
	case MAC_RECV:
		setRxState(MAC_COLL);
		/* fall through */
	case MAC_COLL:
		assert(pktRx_);
		assert(mhRecv_.busy());
		/*
		 *  Since a collision has occurred, figure out
		 *  which packet that caused the collision will
		 *  "last" the longest.  Make this packet,
		 *  pktRx_ and reset the Recv Timer if necessary.
		 */
		if( pktRx_){
			/* sanity 
			if(!mhRecv_.busy()){
				cerr << "weird here2" << endl;
				exit(2);
			}*/

			mhRecv_.stop();
			discard(pktRx_, DROP_MAC_COLLISION);
			pktRx_ = 0;
		} 
		discard(p, DROP_MAC_COLLISION);
		rx_resume();
		break;
	default:
		assert(0);
	}
}

void
Mac802_11::tx_resume()
{
	assert(mhSend_.busy() == 0);
	assert(mhDefer_.busy() == 0);

	// if a node is sleeping, skip the rest
	if( tx_state_ == MAC_SLEEP ) return;

	if(pktCTRL_) {
		/*
		 *  Need to send a ACK.
		 */
		mhDefer_.start(phymib_.getSIFS());
		tx_active_ = 1;
	} else if(pktTx_) {
		if (mhBackoff_.busy() == 0) {
			mhBackoff_.start(cw_, is_idle(), phymib_.getDIFS());
		}
	} else if(callback_) {
		Handler *h = callback_;
		callback_ = 0;
		h->handle((Event*) 0);
	}

	setTxState(MAC_IDLE);
}


void
Mac802_11::rx_resume()
{
	assert(pktRx_ == 0);
	assert(mhRecv_.busy() == 0);


	if(sinrMonitor.channelClear(NOW, CSThresh_)){
		setRxState(MAC_IDLE);
	} else {
		setRxState(MAC_COLL);
		sinrMonitor.start(sinrMonitor.head_->finish_time_ - NOW);
	}
}


/* ======================================================================
   Timer Handler Routines
   ====================================================================== */
void
Mac802_11::backoffHandler()
{
	if(pktCTRL_) {
		assert(mhSend_.busy() || mhDefer_.busy());
		return;
	}


	if(check_pktTx() == 0)
		return;

	// TODO
	// if nothing to do, put a node into sleep? necessary?
	attemptToSleep();
}

void
Mac802_11::deferHandler()
{
	assert(pktCTRL_ || pktTx_);

	if(check_pktCTRL() == 0)
		return;
	assert(mhBackoff_.busy() == 0);
	if(check_pktTx(true) == 0)
		return;
}

void
Mac802_11::navHandler()
{
	if(is_idle() && mhBackoff_.paused())
		mhBackoff_.resume(phymib_.getDIFS());
}

void
Mac802_11::recvHandler()
{
	recv_timer();
}

void
Mac802_11::sendHandler()
{
	send_timer();
}

void Mac802_11::sinrMonitorHandler()
{
	if(sinrMonitor.channelClear(NOW, CSThresh_)){
		setRxState(MAC_IDLE);

	} else {
		setRxState(MAC_COLL);
		sinrMonitor.start(sinrMonitor.head_->finish_time_ - NOW);
	}
}

void
Mac802_11::txHandler()
{
	if (EOTtarget_) {
		assert(eotPacket_);
		EOTtarget_->recv(eotPacket_, (Handler *) 0);
		eotPacket_ = NULL;
	}
	tx_active_ = 0;
}


/* ======================================================================
   The "real" Timer Handler Routines
   ====================================================================== */
void
Mac802_11::send_timer()
{
	switch(tx_state_) {
	/*
	 * Sent DATA, but did not receive an ACK packet.
	 */
	case MAC_SEND:
		RetransmitDATA();
		setTxState(MAC_IDLE, true); // to allow transmission of next data immediately
		break;
	/*
	 * Sent an ACK, and now ready to resume transmission.
	 */
	case MAC_ACK:
		assert(pktCTRL_);
		Packet::free(pktCTRL_); 
		pktCTRL_ = 0;

		break;
	case MAC_IDLE:
		break;
	default:
		assert(0);
	}
	tx_resume();
}


/* ======================================================================
   Outgoing Packet Routines
   ====================================================================== */
int
Mac802_11::check_pktCTRL()
{
	struct hdr_mac802_11 *mh = 0;
	double timeout = 0;

	if(pktCTRL_ == 0)
		return -1;
	if( tx_state_ == MAC_ACK)
		return -1;

	mh = HDR_MAC802_11(pktCTRL_);
							  
	switch(mh->dh_fc.fc_subtype) {
	case MAC_Subtype_ACK:		
		setTxState(MAC_ACK);
		timeout = txtime(phymib_.getACKlen(), basicRate_);
		break;
	default:
		fprintf(stderr, "check_pktCTRL:Invalid MAC Control subtype\n");
		exit(1);
	}
	transmit(pktCTRL_, timeout);
	return 0;
}

int
Mac802_11::check_pktTx(bool ignoremedium)
{
	struct hdr_mac802_11 *mh = 0;
	double timeout = 0;
	
	assert(mhBackoff_.busy() == 0);

	if(pktTx_ == 0)
		return -1;

	mh = HDR_MAC802_11(pktTx_);

	switch(mh->dh_fc.fc_subtype) {
	case MAC_Subtype_Data:
		if(ignoremedium == false && !is_transmission_attempt_allowed()) {
			inc_cw();
			mhBackoff_.start(cw_, is_idle());
			return 0;
		}
		setTxState(MAC_SEND);
		if((u_int32_t)ETHER_ADDR(mh->dh_ra) != MAC_BROADCAST)
                        timeout = txtime(pktTx_)
                                + DSSS_MaxPropagationDelay              // XXX
                               + phymib_.getSIFS()
                               + txtime(phymib_.getACKlen(), basicRate_)
                               + DSSS_MaxPropagationDelay;             // XXX
		else
			timeout = txtime(pktTx_);
		break;
	default:
		fprintf(stderr, "check_pktTx:Invalid MAC Control subtype\n");
		exit(1);
	}
	transmit(pktTx_, timeout);
	return 0;
}

void
Mac802_11::sendACK(int dst)
{
	Packet *p = Packet::alloc();
	hdr_cmn* ch = HDR_CMN(p);
	struct ack_frame *af = (struct ack_frame*)p->access(hdr_mac::offset_);

	assert(pktCTRL_ == 0);

	ch->uid() = 0;
	ch->ptype() = PT_MAC;
	// CHANGE WRT Mike's code
	ch->size() = phymib_.getACKlen();
	ch->iface() = -2;
	ch->error() = 0;
	
	bzero(af, MAC_HDR_LEN);

	af->af_fc.fc_protocol_version = MAC_ProtocolVersion;
 	af->af_fc.fc_type	= MAC_Type_Control;
 	af->af_fc.fc_subtype	= MAC_Subtype_ACK;
 	af->af_fc.fc_to_ds	= 0;
 	af->af_fc.fc_from_ds	= 0;
 	af->af_fc.fc_more_frag	= 0;
 	af->af_fc.fc_retry	= 0;
 	af->af_fc.fc_pwr_mgt	= 0;
 	af->af_fc.fc_more_data	= 0;
 	af->af_fc.fc_wep	= 0;
 	af->af_fc.fc_order	= 0;

	//af->af_duration = ACK_DURATION();
	STORE4BYTE(&dst, (af->af_ra));

	/* store ack tx time */
 	ch->txtime() = txtime(ch->size(), basicRate_);
	
	/* calculate ack duration */
 	af->af_duration = 0;	
	
	pktCTRL_ = p;
}

void
Mac802_11::sendDATA(Packet *p)
{
	hdr_cmn* ch = HDR_CMN(p);
	struct hdr_mac802_11* dh = HDR_MAC802_11(p);

	assert(pktTx_ == 0);

	/*
	 * Update the MAC header
	 */
	ch->size() += phymib_.getHdrLen11();

	dh->dh_fc.fc_protocol_version = MAC_ProtocolVersion;
	dh->dh_fc.fc_type       = MAC_Type_Data;
	dh->dh_fc.fc_subtype    = MAC_Subtype_Data;
	
	dh->dh_fc.fc_to_ds      = 0;
	dh->dh_fc.fc_from_ds    = 0;
	dh->dh_fc.fc_more_frag  = 0;
	dh->dh_fc.fc_retry      = 0;
	dh->dh_fc.fc_pwr_mgt    = 0;
	dh->dh_fc.fc_more_data  = 0;
	dh->dh_fc.fc_wep        = 0;
	dh->dh_fc.fc_order      = 0;

	/* store data tx time */
 	ch->txtime() = txtime(ch->size(), dataRate_);

	if((u_int32_t)ETHER_ADDR(dh->dh_ra) != MAC_BROADCAST) {
		/* store data tx time for unicast packets */
		ch->txtime() = txtime(ch->size(), dataRate_);
		
		dh->dh_duration = usec(txtime(phymib_.getACKlen(), basicRate_)
				       + phymib_.getSIFS());



	} else {
		/* store data tx time for broadcast packets (see 9.6) */
		ch->txtime() = txtime(ch->size(), basicRate_);
		
		dh->dh_duration = 0;
	}
	pktTx_ = p;
}

void
Mac802_11::RetransmitDATA()
{
	struct hdr_cmn *ch;
	struct hdr_mac802_11 *mh;
	u_int32_t *rcount, thresh;
	assert(mhBackoff_.busy() == 0);

	assert(pktTx_);

	ch = HDR_CMN(pktTx_);
	mh = HDR_MAC802_11(pktTx_);

	/*
	 *  Broadcast packets don't get ACKed and therefore
	 *  are never retransmitted.
	 */
	if((u_int32_t)ETHER_ADDR(mh->dh_ra) == MAC_BROADCAST) {
		Packet::free(pktTx_); 
		pktTx_ = 0;

		/*
		 * Backoff at end of TX.
		 */
		rst_cw();
		mhBackoff_.start(cw_, is_idle());

		return;
	}


	macmib_.ACKFailureCount++;

	rcount = &slrc_;
	thresh = macmib_.getLongRetryLimit();

	(*rcount)++;

	if(*rcount >= thresh) {
		/* IEEE Spec section 9.2.3.5 says this should be greater than
		   or equal */
		macmib_.FailedCount++;
		/* tell the callback the send operation failed 
		   before discarding the packet */
		hdr_cmn *ch = HDR_CMN(pktTx_);
		if (ch->xmit_failure_) {
			ch->size() -= phymib_.getHdrLen11();
			ch->xmit_reason_ = XMIT_REASON_ACK;
			ch->xmit_failure_(pktTx_->copy(), ch->xmit_failure_data_);
		}

		*rcount = 0;
		rst_cw();
		discard(pktTx_, DROP_MAC_RETRY_COUNT_EXCEEDED); 
		pktTx_ = 0;
	}
	else {
		struct hdr_mac802_11 *dh;
		dh = HDR_MAC802_11(pktTx_);
		dh->dh_fc.fc_retry = 1;

		inc_cw();
		mhBackoff_.start(cw_, is_idle());
	}
}

/* ======================================================================
   Incoming Packet Routines
   ====================================================================== */
void
Mac802_11::send(Packet *p, Handler *h)
{
	struct hdr_mac802_11* dh = HDR_MAC802_11(p);

	callback_ = h;
	sendDATA(p);
	/*
	 * Assign the data packet a sequence number.
	 */
	dh->dh_scontrol = sta_seqno_++;



	//TODO if medium idle, random backoff, otherwise wait till the end of current transaction
	// node must have waken up already in receive function
	if( is_transmission_attempt_allowed() ){
		if(mhBackoff_.busy() == 0) {
			mhBackoff_.start(cw_, is_idle(), phymib_.getDIFS());
		} else {
			fprintf(stderr, "%d backoff timer is pending1 at %f\n", index_, TIME_NOW);
			exit(1);
		}
	} else {
		// go to sleep and attempt transmission after that or at next wakeup
		if( tx_state_ == MAC_IDLE && rx_state_ == MAC_IDLE && false == mhIdle_.busy()
				)
			attemptToSleep();
	}
}

void
Mac802_11::recv(Packet *p, Handler *h)
{
	struct hdr_cmn *hdr = HDR_CMN(p);
	/*
	 * Sanity Check
	 */
	assert(initialized()) ;

	/*
	 *  Handle outgoing packets.
	 */
	if(hdr->direction() == hdr_cmn::DOWN) {

		// if radio is sleeping, wakeup
		if( tx_state_ == MAC_SLEEP ){
			wakeup();

		}

		send(p, h);
		return;
	}
	/*
	 *  Handle incoming packets.
	 *
	 *  We just received the 1st bit of a packet on the network
	 *  interface.
	 *
	 */

	if( hdr->below_cs_threshold_ ){
		sinrMonitor.add(txtime(p), p, NOW);
		Packet::free(p);
		if(false == sinrMonitor.channelClear(NOW, CSThresh_)){
			if( mhBackoff_.busy() && ! mhBackoff_.paused() )
				mhBackoff_.pause();
		}

		return;
	}


	// if node is sleeping
	if( tx_state_ == MAC_SLEEP ){
		sinrMonitor.add(txtime(p), p, NOW);
		Packet::free(p);
		return;
	}

	/*
	 *  If the interface is currently in transmit mode, then
	 *  it probably won't even see this packet.  However, the
	 *  "air" around me is BUSY so I need to let the packet
	 *  proceed.  Just set the error flag in the common header
	 *  to that the packet gets thrown away.
	 */
	if(tx_active_ && hdr->error() == 0) {
		hdr->error() = 1;
	}

	hdr->arrival_time_ = NOW;

	if( mhBackoff_.busy() && !mhBackoff_.paused() )
		mhBackoff_.pause();

	
	if(rx_state_ == MAC_IDLE) {

		setRxState(MAC_RECV);
		pktRx_ = p;
		/*
		 * Schedule the reception of this packet, in
		 * txtime seconds.
		 */
		mhRecv_.start(txtime(p));
		sinrMonitor.add(txtime(pktRx_), pktRx_, NOW);
		
	} else {

		if( sinrMonitor.busy() ){
			sinrMonitor.stop();
		}

#ifdef MAC_WITH_CAPTURE // if capture effect is enabled
		/*
		 *  If the power of the incoming packet is smaller than the
		 *  power of the packet currently being received by at least
                 *  the capture threshold, then we ignore the new packet.
		 */
		// use accumulative energy instead of p's energy
		if(pktRx_ && pktRx_->txinfo_.RxPr / (sinrMonitor.getTotal(NOW) - pktRx_->txinfo_.RxPr + p->txinfo_.RxPr) >= p->txinfo_.CPThresh) {
			sinrMonitor.add(txtime(p), p, NOW);
			// keep original rx state
			capture(p);
		} 
		else if(/*false == isPLCPHdrDone(pktRx_) && */p->txinfo_.RxPr / sinrMonitor.getTotal(NOW) >= p->txinfo_.CPThresh){
			sinrMonitor.add(txtime(p), p, NOW);
			if( pktRx_ ){
				capture(pktRx_); 
				mhRecv_.stop();
			} 
			setRxState(MAC_RECV);
			pktRx_ = p;
			mhRecv_.start(txtime(pktRx_));
		}

		else
#endif //MAC_WITH_CAPTURE
		{
			sinrMonitor.add(txtime(p), p,NOW);
			collision(p);
		}
	}
}

void
Mac802_11::recv_timer()
{
	//u_int32_t src; 
	hdr_cmn *ch = HDR_CMN(pktRx_);
	hdr_mac802_11 *mh = HDR_MAC802_11(pktRx_);
	u_int32_t dst = ETHER_ADDR(mh->dh_ra);
	
	u_int8_t  type = mh->dh_fc.fc_type;
	u_int8_t  subtype = mh->dh_fc.fc_subtype;

	assert(pktRx_);
	assert(rx_state_ == MAC_RECV || rx_state_ == MAC_COLL);
	
        /*
         *  If the interface is in TRANSMIT mode when this packet
         *  "arrives", then I would never have seen it and should
         *  do a silent discard without adjusting the NAV.
         */
        if(tx_active_) {
                Packet::free(pktRx_);
                goto done;
        }

	/*
	 * Handle collisions.
	 */
	if(rx_state_ == MAC_COLL) {
		fprintf(stderr, "Mac802_11:recv_timer collided pkts should already have been dropped\n");
		exit(1);
		discard(pktRx_, DROP_MAC_COLLISION);		

		goto done;
	}

	/*
	 * Check to see if this packet was received with enough
	 * bit errors that the current level of FEC still could not
	 * fix all of the problems - ie; after FEC, the checksum still
	 * failed.
	 */
	if( ch->error() ) {
		Packet::free(pktRx_);
		goto done;
	}

	/*
	 * IEEE 802.11 specs, section 9.2.5.6
	 *	- update the NAV (Network Allocation Vector)
	 */
        /* tap out - */
        if (tap_ && type == MAC_Type_Data &&
            MAC_Subtype_Data == subtype ) 
		tap_->tap(pktRx_);

	/*
	 * Address Filtering
	 */
	if(dst != (u_int32_t)index_ && dst != MAC_BROADCAST) {
		/*
		 *  We don't want to log this event, so we just free
		 *  the packet instead of calling the drop routine.
		 */
		discard(pktRx_, "---");
		goto done;
	}


	switch(type) {

	case MAC_Type_Management:
		discard(pktRx_, DROP_MAC_PACKET_ERROR);
		goto done;
	case MAC_Type_Control:
		switch(subtype) {
		case MAC_Subtype_ACK:
			recvACK(pktRx_);
			break;
		default:
			fprintf(stderr,"recvTimer1:Invalid MAC Control Subtype %x\n",
				subtype);
			exit(1);
		}
		break;
	case MAC_Type_Data:
		switch(subtype) {
		case MAC_Subtype_Data:
			recvDATA(pktRx_);
			break;
		default:
			fprintf(stderr, "recv_timer2:Invalid MAC Data Subtype %x\n",
				subtype);
			exit(1);
		}
		break;
	default:
		fprintf(stderr, "recv_timer3:Invalid MAC Type %x\n", subtype);
		exit(1);
	}
 done:
	pktRx_ = 0;
	rx_resume();
}



/*
 * txtime()	- pluck the precomputed tx time from the packet header
 */
double
Mac802_11::txtime(Packet *p)
{
	struct hdr_cmn *ch = HDR_CMN(p);
	double t = ch->txtime();
	if (t < 0.0) {
		drop(p, "XXX");
 		exit(1);
	}
	return t;
}

 
/*
 * txtime()	- calculate tx time for packet of size "psz" bytes 
 *		  at rate "drt" bps
 */
double
Mac802_11::txtime(double psz, double drt)
{
	double dsz = psz - phymib_.getPLCPhdrLen();
        int plcp_hdr = phymib_.getPLCPhdrLen() << 3;	
	int datalen = (int)dsz << 3;
	double t = (((double)plcp_hdr)/phymib_.getPLCPDataRate())
                                       + (((double)datalen)/drt);
	return(t);
}



void
Mac802_11::recvDATA(Packet *p)
{
	struct hdr_mac802_11 *dh = HDR_MAC802_11(p);
	u_int32_t dst, src, size;
	struct hdr_cmn *ch = HDR_CMN(p);

	dst = ETHER_ADDR(dh->dh_ra);
	src = ETHER_ADDR(dh->dh_ta);
	size = ch->size();
	/*
	 * Adjust the MAC packet size - ie; strip
	 * off the mac header
	 */
	ch->size() -= phymib_.getHdrLen11();
	ch->num_forwards() += 1;

	if(dst != MAC_BROADCAST) {
		sendACK(src);
		if(mhSend_.busy() == 0)
			tx_resume();
	}
	
	/* ============================================================
	   Make/update an entry in our sequence number cache.
	   ============================================================ */

	/* Changed by Debojyoti Dutta. This upper loop of if{}else was 
	   suggested by Joerg Diederich <dieder@ibr.cs.tu-bs.de>. 
	   Changed on 19th Oct'2000 */

        if(dst != MAC_BROADCAST) {
                if (src < (u_int32_t) cache_node_count_) {
                        Host *h = &cache_[src];

                        if(h->seqno && h->seqno == dh->dh_scontrol) {
                                discard(p, DROP_MAC_DUPLICATE);
                                return;
                        }
                        h->seqno = dh->dh_scontrol;
                } else {
			static int count = 0;
			if (++count <= 10) {
				printf ("MAC_802_11: accessing MAC cache_ array out of range (src %u, dst %u, size %d)!\n", src, dst, cache_node_count_);
				if (count == 10)
					printf ("[suppressing additional MAC cache_ warnings]\n");
			};
		};
	}

	uptarget_->recv(p, (Handler*) 0);
}


void
Mac802_11::recvACK(Packet *p)
{	
	if(tx_state_ != MAC_SEND) {
		discard(p, DROP_MAC_INVALID_STATE);
		return;
	}
	assert(pktTx_);
	mhSend_.stop();

	/*
	 * The successful reception of this ACK packet implies
	 * that our DATA transmission was successful.  Hence,
	 * we can reset the Short/Long Retry Count and the CW.
	 *
	 * need to check the size of the packet we sent that's being
	 * ACK'd, not the size of the ACK packet.
	 */
	slrc_ = 0;
	rst_cw();

	/*
	 * Backoff before sending again.
	 */
	assert(mhBackoff_.busy() == 0);
	mhBackoff_.start(cw_, is_idle());

	Packet::free(pktTx_); 
	pktTx_ = 0;

	tx_resume();

	mac_log(p);

}

void MACHelloTimer::start(int nodeid, double delay)
{
	if( busy_ ){
		fprintf(stderr, "wakeuptimer is pending while start() is called at %f\n", TIME_NOW);
		exit(1);
	}

	busy_ = true;
	// enforce that the drift ends within a dutycycle
	double offset = delay; 
	Scheduler::instance().schedule(this, &intr, offset );
}

void MACHelloTimer::handle(Event *)
{
	busy_ = false;
	mac->handleWakeup();
}



void Mac802_11::start_hello(){
	hellotimer_ = new MACHelloTimer(this);
	// start at a random time between 1. and 9.
	hellotimer_->start(index_, Random::uniform(1.,9.));
}

void Mac802_11::start_neighbourdetect(){
	myneighbor_list.neighbor_cnt_ = 0;
	myneighbor_list.head = 0;
}





void Mac802_11::recv_beacon(u_int32_t nodemac)
{
	myneighbor_list_item *np;
	np = myneighbor_list.head;
	for (; np; np = np->next) {
		if (np->id == nodemac) {
	//		np->etx_curindex_ = (np->etx_curindex_+1) % ETX_HISTORY_LENGTH ;
	//		np->etx_history_[np->etx_curindex_] = NOW;
			break;
		}
	}
	if (!np) {      // insert this new entry
		np = new myneighbor_list_item(index_);
		np->id = nodemac;
		np->next = myneighbor_list.head;
		myneighbor_list.head = np;
		myneighbor_list.neighbor_cnt_++;
	}
}

myneighbor_list_item* Mac802_11::getNeighbor(u_int32_t dmac){
	myneighbor_list_item *np;
	np = myneighbor_list.head;
	for (; np; np = np->next) {
		if (np->id == dmac) {
			// drops within a given perios
			return np;
		}
	}
	return NULL;
}

/*output to trace file regardless of the verbose level */
void Mac802_11::direct_trace(char* fmt, ...)
{
  va_list ap;
  
  if (!logtarget_) return;

  va_start(ap, fmt);
  vsprintf(logtarget_->pt_->buffer(), fmt, ap);
  logtarget_->pt_->dump();
  va_end(ap);
}

myneighbor_list_item::myneighbor_list_item(int index, u_int32_t initCW){
	id = 90000;
	macindex_ = index;
	next = NULL;
	transmission_bound_time_ = 0.;
	if_collision_ = false;
	cw_ = initCW;
}


myneighbor_list_item::~myneighbor_list_item(){
}

SINRMonitor::SINRMonitor(Mac802_11 *m) : MacTimer(m) {
	head_ = 0;
	count_ = 0;
	totalpw_ = 0;
	time_of_last_bit_ = 0;
	mac_ = m;
}
void SINRMonitor::cleanup(double curtime){
	// increase curtime a little bit to handle possible rounding problem in floating point calculation
	curtime += 1.0e-6;

	if( count_ == 0 ){
		return;
	} else {
		if( curtime >= head_->finish_time_ ){
			SignalFinishTime *cur = head_;
			while( head_ && curtime >= head_->finish_time_ ){
				cur = head_;
				head_ = head_->next_;
				totalpw_ -= cur->strength_;
				count_--;
				if( count_ == 0 )
					totalpw_ = 0.; // to handle rounding errors
				delete cur;
			}
		}
	}
}

double SINRMonitor::getTotal(double curtime){
	// two signal may cancel each other some how // TODO more detailed model
	// debug
	if( count_ == 0 ){
		return 0;
	} else {
		cleanup(curtime);
	}

	return totalpw_;
}

bool SINRMonitor::channelClear(double curtime, double CSThresh){
//	cout << "SINRMonitor::channelClear now " << curtime << " time_of_last_bit_ " << time_of_last_bit_ << endl;
	if( getTotal(curtime) >= CSThresh )
		return false;
	else 
		return true;
}
void    
SINRMonitor::handle(Event *)
{       
	busy_ = 0;
	paused_ = 0;
	stime = 0.0;
	rtime = 0.0;

	mac->sinrMonitorHandler();
}

void SINRMonitor::add(double transmissiontime, Packet *p, double curtime){

	cleanup(curtime);

	double finishtime = transmissiontime + curtime;
	double signalstrength = p->txinfo_.RxPr;

	if( finishtime > time_of_last_bit_ )
		time_of_last_bit_ = finishtime;

	SignalFinishTime *entry = new SignalFinishTime(finishtime, signalstrength);

	totalpw_ += signalstrength;

	if( count_ == 0 ){
		count_++;
		head_ = entry;
		return;
	} else {
		count_++;
		SignalFinishTime *prev;
		// insert to the head
		if( finishtime < head_->finish_time_) {
			entry->next_ = head_;
			head_ = entry;
			return;
		} else {
			prev = head_;
		}

		SignalFinishTime *cur = prev->next_;
		while( cur && finishtime > cur->finish_time_){
			prev = cur;
			cur = prev->next_;
		}
		entry->next_ = cur;
		prev->next_ = entry;
	}
}

bool Mac802_11::is_transmission_attempt_allowed(bool is_xmac_preamble){
	// states other than tx_state
	bool otherviolation = rx_state_ != MAC_IDLE ;
	if( otherviolation ){
violated:
		return false;
	}

	if( tx_state_ != MAC_IDLE ){
		goto violated;
	}
	return true;
}

// power state control
bool Mac802_11::attemptToSleep(bool forced) 
{
	if( forced == false ){
		if_want_sleep_ = true;

		// check if any timer is active
		if(mhSend_.busy() || mhRecv_.busy() || mhDefer_.busy() ||
				mhBackoff_.busy() || mhIF_.busy() || mhIdle_.busy() ||
				rx_state_ != MAC_IDLE || tx_state_ != MAC_IDLE ){
			return false;
		}
		if_want_sleep_ = false;
	}


	hellotimer_->start(index_, duty_cycle_length_);

	// dutycycle counter
	if( tx_state_ != MAC_SLEEP ){
		if(duty_cycle_stat_started_){
			God::instance()->total_active_time_ += TIME_NOW - last_radio_state_update_time_;
			God::instance()->duty_cycle_active_[index_] += TIME_NOW - last_radio_state_update_time_;
		} else if( TIME_NOW >= START_MEASUREMENT_THRESHOLD ){
			duty_cycle_stat_started_ = true;
			God::instance()->total_active_time_ += TIME_NOW - START_MEASUREMENT_THRESHOLD;
			God::instance()->duty_cycle_active_[index_] += TIME_NOW - START_MEASUREMENT_THRESHOLD;
		}
		last_radio_state_update_time_ = TIME_NOW;
	}

	// TODO cancel or wait all pending transactions
	// go to sleep, turn off radio
	tx_state_ = rx_state_ = MAC_SLEEP;

	((WirelessPhy *)netif_)->node_sleep();
	return true;
}

bool Mac802_11::wakeup()
{
	// dutycycle counter
	if( tx_state_ == MAC_SLEEP ){
		if(duty_cycle_stat_started_){
			God::instance()->total_sleep_time_ += TIME_NOW - last_radio_state_update_time_;
			God::instance()->duty_cycle_sleep_[index_] += TIME_NOW - last_radio_state_update_time_;
		} else if( TIME_NOW >= START_MEASUREMENT_THRESHOLD ){
			duty_cycle_stat_started_ = true;
			God::instance()->total_sleep_time_ += TIME_NOW - START_MEASUREMENT_THRESHOLD;
			God::instance()->duty_cycle_sleep_[index_] += TIME_NOW - START_MEASUREMENT_THRESHOLD;
		}
		last_radio_state_update_time_ = TIME_NOW;
	}
	//wakeup from sleep. turn on radio
	// this is the only place tx_state_ and rx_state_ can be changed from sleep to other state
	tx_state_ = rx_state_ = MAC_IDLE;

	//TODO: consider wakeup delay, 2ms for mica2 radio


	((WirelessPhy *) netif_)->node_wakeup();

	return true;
}

void Mac802_11::handleWakeup(){
	fprintf(stderr, "handleWakeup should not be called\n");
	exit(1);
}
void IdleGuardTimer::start(double time)
{
	Scheduler &s = Scheduler::instance();
	assert(busy_ == 0);
	busy_ = 1;
	paused_ = 0;
	stime = s.clock();
	rtime = time;
	assert(rtime >= 0.0);

	s.schedule(this, &intr, rtime);
}


void IdleGuardTimer::handle(Event *)
{       
	busy_ = 0;
	paused_ = 0;
	stime = 0.0;
	rtime = 0.0;

	mac->idleGuardTimerHandler();
}

void Mac802_11::idleGuardTimerHandler(){
	fprintf(stderr, "idleGuardTimerHandler should not be called\n");
	exit(1);
}
void Mac802_11::decideWhetherToSleep(){
	fprintf(stderr, "decideWhetherToSleep should not be called\n");
	exit(1);
}

#ifdef YJ_QUEUED_PKT_WAITER
void QedPktWaitTimer::start(double time)
{
	Scheduler &s = Scheduler::instance();
	assert(busy_ == 0);
	busy_ = 1;
	paused_ = 0;
	stime = s.clock();
	rtime = time;
	assert(rtime >= 0.0);

	s.schedule(this, &intr, rtime);
}


void QedPktWaitTimer::handle(Event *)
{       
	busy_ = 0;
	paused_ = 0;
	stime = 0.0;
	rtime = 0.0;

	mac->QedPktWaitTimerHandler();
}

void Mac802_11::QedPktWaitTimerHandler(){
	// sanity check
	if( tx_state_ == MAC_SLEEP ){
		fprintf(stderr, "ERROR!!! %d QedPktWaitTimerHandler: node already in sleep at %f\n", index_, TIME_NOW);
		exit(1);
	}
	attemptToSleep(true);
}
#endif




