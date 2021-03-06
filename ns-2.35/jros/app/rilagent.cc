

#include "rilagent.h"
#include <iostream>
static class RILAgentClass : public TclClass {
public:
	RILAgentClass() : TclClass("Agent/RILAgent") {}
	TclObject* create(int, const char*const*) {
		return (new RILAgent());
	}
}class_RILAgent;

RILAgent::RILAgent() : Agent(PT_RILAGENT), seqno_(-1), lastSeq(-1), lastTime(-1.0) {
	bind("packetSize_", &size_);
}

RILAgent::RILAgent(packet_t type) : Agent(type), lastSeq(-1), lastTime(-1.0) {
	bind("packetSize_", &size_);
}

void RILAgent::stop() {

}

//commonherader|ipheader|packetdata+appdata header|nrrapp header
//
void RILAgent::sendmsg(int nbytes, AppData* data, const char* flags) {
	int n = nbytes;
	assert (size_ > 0);
	if (n == -1) {
		return;
	}
	// If they are sending data, then it must fit within a single packet.
	if (n > size_) {
		return;
	}

	//double local_time = NOWTIME;
	PacketData* pd = (PacketData*)data;
	RILAppMessageT_* cappmsgp = (RILAppMessageT_*)pd->data();
	Packet* p = 0;
	if (n > 0) {
		// allocate a packet
		p = allocpkt();
		// fill common header content
		hdr_cmn *ch = hdr_cmn::access(p);
		ch->size() = n;
		ch->timestamp() = (u_int32_t)(NOW);
		ch->ptype() = PT_RILAGENT;
		ch->num_forwards() = 0;
        ch->prev_hop_ = here_.addr_;
		// fill ip header content
		hdr_ip *iph = hdr_ip::access(p);
	    iph->ttl() = 100;
	    iph->saddr() = addr();
	    iph->sport() = 0;
	    iph->dport() = 0;
	    // check nrrmsg net type to update the ip header
	    if (cappmsgp->netType == MCAST || cappmsgp->netType == BCAST) {
	    	iph->daddr() = IP_BROADCAST;
		    iph->sport() = 0;
		    iph->dport() = 0;
	    }
	    else {
	    	iph->daddr() = cappmsgp->dest;
	    }
	    p->setdata(data);
		target_->recv(p);
	}
	idle();
}
void RILAgent::recv(Packet* pkt, Handler* h) {
	if (app_ ) {
		hdr_cmn* ch = hdr_cmn::access(pkt);
		hdr_ip* ih = hdr_ip::access(pkt);
		// this data may include unicast msg to me, snooped unicast msg to others and bcast msg
		if (ih->daddr() == here_.addr_) {
			app_->process_data(ch->size(), pkt->userdata());
			Packet::free(pkt);
		}
        else if (ch->next_hop_ == here_.addr_) {
            // snooped
            //printf("node %d snoop from %d %d\n",here_.addr_, ch->prev_hop_, pkt);
            app_->process_data(ch->size(), pkt->userdata());
			//Packet::free(pkt);
        }
#if 0
		// this is a new bcast msg, need to rebroadcast
		else if (ih->daddr() == IP_BROADCAST && ih->saddr() != addr() && msg->seq > lastSeq && msg->timestamp != lastTime) {
			lastSeq = msg->seq;
			lastTime = msg->timestamp;
			ch->direction() = hdr_cmn::DOWN;
			app_->process_data(ch->size(), pkt->userdata()); // signal up
			target_->recv(pkt);// rebroadcast
		}
#endif
		// if this is not a unicast message sent to me, i cannot free it here
	} else {
		// don't process it if agent is null
		Packet::free(pkt);
	}
}


int RILAgent::command(int argc, const char*const* argv) {
	Tcl& tcl = Tcl::instance();
	if (argc == 3) {
		if (strcmp(argv[1], "connect-agent") == 0) {
			Agent* a=0;
			a = (Agent*)tcl.lookup(argv[2]);
			if (a!=0) {
				std::cout<<a->addr()<<std::endl;
				std::cout<<a->port()<<std::endl;
			}
			dst_.addr_ = a->addr();
			dst_.port_ = a->port();
			return (TCL_OK);
		}
	}
	return (Agent::command(argc, argv));
}

