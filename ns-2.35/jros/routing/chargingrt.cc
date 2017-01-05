 /*
 * nrrrouting.cc
 *
 *  Created on: Sep 21, 2009
 *      Author: ypeng
 */

#include "chargingrt.h"
#include <vector>
using namespace std;

int hdr_chargingrt_pkt::offset_;
static class ChargingRTHeaderClass:public PacketHeaderClass {
public:
	ChargingRTHeaderClass():
		PacketHeaderClass("PacketHeader/ChargingRT", sizeof(hdr_chargingrt_pkt)) {
		bind_offset(&hdr_chargingrt_pkt::offset_);
	}
}class_chartingrt_hdr;

static class ChargingRTClass : public TclClass {
public:
    ChargingRTClass() : TclClass("Agent/ChargingRT") {}
    TclObject* create(int argc, const char*const* argv) {
        return (new ChargingRT((nsaddr_t)Address::instance().str2addr(argv[4])));
    }
}class_chargingrt;

ChargingRT::ChargingRT(nsaddr_t id) : Agent(PT_CHARGINGRT),
		ra_addr_(id),seq_num_(0),packetSent_(0),
        parentID_(-1), sinkID_(0), hopCnt_(0xffff),myCost_(MAX_COST){
	bind("RTYPE", &routingType_);
    int temp = 0;
    bind("e2edelayreq", &temp); e2ereq_ = temp*1.0;
    bind("routinginterval",&routinginterval_);
    lastUpdate_ = NOW;
	btimer = new RTBeaconTimer(this);
}

ChargingRT::~ChargingRT(){
	if (btimer) btimer->force_cancel();
	delete btimer;
    btimer = 0;
}

int ChargingRT::command(int argc, const char*const* argv) {
	Tcl& tcl = Tcl::instance();
	if(argc == 2) {
		if(strncasecmp(argv[1], "id", 2) == 0) {
		  return TCL_OK;
		}
		else if(strncasecmp(argv[1], "start", 2) == 0) {
			if (btimer) btimer->sched(1.0+routinginterval_);
			printf("node %d x %f y %f \n",ra_addr_, mnode->X(),mnode->Y());
			return TCL_OK;
		 }
	  }
	  else if(argc == 3) {
		if(strcmp(argv[1], "log-target") == 0 || strcmp(argv[1], "tracetarget") == 0) {
		   logtarget_ = (Trace*) TclObject::lookup(argv[2]);
		   if(logtarget_ == 0) return TCL_ERROR;
		   return TCL_OK;
		}
		else if (strcmp(argv[1], "port-dmux") == 0) {
			dmux_ = (PortClassifier *)TclObject::lookup(argv[2]);
			if (dmux_ == 0) {
				fprintf (stderr, "%s: %s lookup of %s failed\n", __FILE__, argv[1], argv[2]);
				return TCL_ERROR;
			}
			return TCL_OK;
		}
        else if (strcmp(argv[1], "add-node") == 0) {
			mnode = (MobileNode*)tcl.lookup(argv[2]);
            if (mnode!=0) {return TCL_OK;}
            else return TCL_ERROR;
        }
	  }
	  return Agent::command(argc, argv);
}

void ChargingRT::updateNeightorTable(neighborT_& node) {
    if (God::instance()->isSink(ra_addr_)) return;
	list<neighborT_>::iterator itl = neighborL.begin();
	list<neighborT_>::iterator end = neighborL.end();
    while (itl!=end){
		if (itl->id == node.id) {
			itl->x = node.x;
			itl->y = node.y;
            itl->energy = node.energy;
            itl->cost = node.cost;
            itl->sinkID = node.sinkID;
            itl->hopCnt = node.hopCnt;
            itl->timestamp = node.timestamp;
            itl->parent = node.parent;
            if (itl->energy<1) {neighborL.erase(itl);}
			return;
		}
		itl++;
	}
    if (node.energy>1){
        neighborL.push_back(node);
    }
}

void ChargingRT::evictNeighbor(int nodeid) {
	list<neighborT_>::iterator itl = neighborL.begin();
	list<neighborT_>::iterator end = neighborL.end();
    while (itl!=end){
		if (itl->id == nodeid) {
            neighborL.erase(itl);
			return;
		}
		itl++;
	}
}

void ChargingRT::updateNeighborTime(int nodeid) {
	list<neighborT_>::iterator itl = neighborL.begin();
	list<neighborT_>::iterator end = neighborL.end();
    while (itl!=end){
		if (itl->id == nodeid) {
            itl->timestamp = NOW;
            itl->parent = ra_addr_;
            itl->sinkID = sinkID_;
			return;
		}
		itl++;
	}
}

double ChargingRT::getMinHop(int* sinkID, int* minid) {
	list<neighborT_>::iterator it = neighborL.begin();
	list<neighborT_>::iterator end = neighborL.end();
    double curcost = MAX_COST;
    *sinkID = sinkID_; *minid = -1;
	while (it!=end) {
        if (it->energy<1) {it++;continue;}
		    double cost = it->hopCnt + 1;
            if (cost < curcost) {
                curcost = cost;
                *minid = it->id;
                sinkID_ = it->sinkID;
                *sinkID = sinkID_;
                hopCnt_ = it->hopCnt+1;
            }
        it++;
	}
    myCost_ = curcost;
    return curcost;
}

double ChargingRT::getMinCost(int* sinkID, int* minid) {
	list<neighborT_>::iterator it = neighborL.begin();
	list<neighborT_>::iterator end = neighborL.end();
    double curcost = MAX_COST;
    *sinkID = sinkID_; *minid = -1;
    double leftratio = em()->energy()/MAX_ENERGY;
    double pktCost = 0.1;
    if ( leftratio > 0.93 || leftratio < 0.07) {pktCost = 1;}
    else if (leftratio > 0.85 || leftratio < 0.15) {pktCost = 0.5;}
    else pktCost = 0.1;
	while (it!=end) {
        if (it->energy<1) {it++;continue;}
        if (it->timestamp + 10.0*beaconInterval_ < NOW || it->parent == ra_addr_) {
            it++;
            continue;
        }
	    double cost = it->cost;
        if (God::instance()->isSink(it->id)) {
            curcost = cost;
            *minid = it->id;
            sinkID_ = it->sinkID;
            *sinkID = sinkID_;
            hopCnt_ = it->hopCnt+1;
            break;
        }
        if (cost < curcost) {
            curcost = cost;
            *minid = it->id;
            sinkID_ = it->sinkID;
            *sinkID = sinkID_;
            hopCnt_ = it->hopCnt+1;
        }
        it++;
	}
    myCost_ =  curcost+ pktCost*pow(10000, 1-em()->energy()/MAX_ENERGY);//curcost;
    return curcost;
}

double ChargingRT::getMinDist(int* sinkID, int* minid) {
    list<neighborT_>::iterator it = neighborL.begin();
    list<neighborT_>::iterator end = neighborL.end();
    double curcost = MAX_COST;
    *sinkID = sinkID_; *minid = -1;
    while (it!=end) {
        if (it->energy<1) {it++;continue;}
        if (it->timestamp + 10.0*beaconInterval_ < NOW || it->parent == ra_addr_) {
            it++;
            continue;
        }
        double ndis = it->cost + pow(it->x-mnode->X(),2)+pow(it->y-mnode->Y(),2);

        if (God::instance()->isSink(it->id)) {
            curcost = ndis;
            *minid = it->id;
            sinkID_ = it->sinkID;
            *sinkID = sinkID_;
            hopCnt_ = it->hopCnt+1;
            break;
        }
        if (ndis+0.1 < curcost) {
            curcost = ndis;
            *minid = it->id;
            sinkID_ = it->sinkID;
            *sinkID = sinkID_;
            hopCnt_ = it->hopCnt+1;
        }
	    it++;
    }
    myCost_ = curcost;
    return curcost;
}

double ChargingRT::getEA(int* sinkID, int* nextid) {
    int neiid = 0;
    double mydistance = pow(node_xp[ra_addr_],2)+pow(node_yp[ra_addr_],2);
    *sinkID = 0; *nextid = parentID_;
    if (parentID_<= 0) {
        neiid = ntbl[ra_addr_][0];
        *nextid = neiid;
        myCost_ = God::instance()->min_upstream_life[neiid];
        God::instance()->current_parent[ra_addr_] = neiid;
        return myCost_;
    }
    if (lastUpdate_ + routinginterval_ > NOW) return myCost_;
    lastUpdate_ = NOW;

    myCost_ = God::instance()->my_lifetime[parentID_];
    //myCost_ = God::instance()->min_upstream_life[parentID_];
myCost_ = God::instance()->my_energy[parentID_];
    for (int i=0; i< maxneighbor; i++) {
        neiid = ntbl[ra_addr_][i];
        double neidistance = pow(node_xp[neiid],2)+pow(node_yp[neiid],2);
        double minlife;
        if (neiid != -1 && neidistance+0.001 < mydistance && neiid<God::instance()->nodes() && !God::instance()->my_disconnection[neiid]) {
            if (God::instance()->isSink(neiid)) {
                myCost_ = God::instance()->my_lifetime[neiid];
                //myCost_ = God::instance()->min_upstream_life[neiid];
myCost_ = God::instance()->my_energy[neiid];
                *nextid = neiid;
                sinkID_ = 0;
                *sinkID = sinkID_;
                God::instance()->current_parent[ra_addr_] = neiid;
                break;
            }
            else {
                //double delay = God::instance()->max_upstream_delay[neiid]+ God::instance()->max_downstream_delay[ra_addr_]+God::instance()->my_tr[neiid];
                minlife = God::instance()->my_lifetime[neiid];
                //minlife = God::instance()->min_upstream_life[neiid];
minlife = God::instance()->my_energy[neiid];
                if (minlife > myCost_ + 1
                    //&& delay < e2ereq_+0.001
                    && God::instance()->current_parent[neiid]!=ra_addr_) {
                    myCost_ = minlife;
                    *nextid = neiid;
                    sinkID_ = 0;
                    *sinkID = sinkID_;
                    God::instance()->current_parent[ra_addr_] = neiid;
                }
            }
        }
    }
    return myCost_;
}

double ChargingRT::getEAPlus(int* sinkID, int* nextid) {
    int neiid = 0;
    *sinkID = 0; *nextid = parentID_;
    if (parentID_<= 0) {
        neiid = ntbl[ra_addr_][0];
        *nextid = neiid;
        God::instance()->current_parent[ra_addr_] = neiid;
        myCost_ = God::instance()->min_upstream_life[neiid];
        return myCost_;
    }
    if (lastUpdate_+routinginterval_ >NOW) return myCost_;
    lastUpdate_ = NOW;

    double extra, newparentlife, curparentlife, mylife;
    myCost_ = God::instance()->my_lifetime[parentID_];
    if (myCost_ > God::instance()->my_lifetime[ra_addr_]) myCost_ = God::instance()->my_lifetime[ra_addr_];
    double tempcost=myCost_;
    double mydistance = pow(node_xp[ra_addr_],2)+pow(node_yp[ra_addr_],2);

    for (int i=0; i < maxneighbor; i++) {
        neiid = ntbl[ra_addr_][i];
        double neidistance = pow(node_xp[neiid],2)+pow(node_yp[neiid],2);
        if (neiid != -1 && neidistance+0.001 < mydistance && neiid != parentID_ && neiid<God::instance()->nodes()) {
            if (God::instance()->isSink(neiid)) {
                myCost_ = God::instance()->min_upstream_life[neiid];
                *nextid = neiid;
                sinkID_ = 0;
                *sinkID = sinkID_;
                God::instance()->current_parent[ra_addr_] = neiid;
                break;
            }
            else {
                if (//God::instance()->current_parent[neiid]!=parentID_ &&
                    God::instance()->current_parent[neiid]!=ra_addr_
                    ) {
                    double extra_cost = 0;
                    bool statechanged = false;
                    int minid = neiid; int oldminid = parentID_;
                    //minid = God::instance()->min_upstream_id[neiid]; oldminid = God::instance()->min_upstream_id[parentID_];
                    extra = e2ereq_ - God::instance()->max_upstream_delay[neiid] - God::instance()->max_downstream_delay[ra_addr_]-God::instance()->my_tr[neiid];
                    newparentlife = God::instance()->my_lifetime[minid];
                    mylife = God::instance()->my_lifetime[ra_addr_];
                    curparentlife = God::instance()->my_lifetime[oldminid];

                    if (newparentlife < curparentlife +10 && newparentlife < mylife+10) continue;

                    extra_cost = God::instance()->my_cr[oldminid] - God::instance()->my_outrate[ra_addr_]*God::instance()->my_rxtr[oldminid]*0.0588;
                    if (extra_cost < 0.001) extra_cost = 0.001;
                    curparentlife = God::instance()->my_energy[oldminid]/(extra_cost*60);

                    double amendedextra = extra;

                    if (newparentlife > mylife+10) {
                        // estimate lifetime of myself,
                        if (extra > 0) {
                            extra_cost = God::instance()->my_cr[minid]+God::instance()->my_outrate[ra_addr_]*God::instance()->my_rxtr[minid]*0.0588;
                            newparentlife = God::instance()->my_energy[minid]/(extra_cost*60);
                            
                            extra_cost = God::instance()->my_cr[ra_addr_];
                                       //- God::instance()->my_tr[ra_addr_]*amendedextra*0.005/((God::instance()->my_tr[ra_addr_]+amendedextra)*God::instance()->my_tr[ra_addr_]);
                            amendedextra = God::instance()->my_tr[neiid]-God::instance()->my_rxtr[ra_addr_];
                            if (amendedextra > 0) amendedextra = 0;
                            extra_cost += God::instance()->my_outrate[ra_addr_]*amendedextra*0.0588;
                        } else {
                            amendedextra = extra/God::instance()->current_hop[neiid];
                            if (God::instance()->my_tr[minid]+amendedextra<0.01) extra_cost = God::instance()->my_cr[minid] + (1-0.01/God::instance()->my_tr[minid])*0.0588;
                            else extra_cost = God::instance()->my_cr[minid] -
                                              God::instance()->my_tr[minid]*amendedextra*0.01*0.0588/((God::instance()->my_tr[minid]+amendedextra)*God::instance()->my_tr[minid]);
                            extra_cost += God::instance()->my_outrate[ra_addr_]*God::instance()->my_rxtr[minid]*0.0588;
                            newparentlife = God::instance()->my_energy[minid]/(extra_cost*60);
                            
                            extra_cost = God::instance()->my_cr[ra_addr_];
                            amendedextra = God::instance()->my_tr[neiid]-God::instance()->my_rxtr[ra_addr_]+extra/God::instance()->current_hop[neiid];
                            //if (amendedextra > 0) amendedextra = 0;
                            extra_cost += God::instance()->my_outrate[ra_addr_]*amendedextra*0.0588;
                        }
                        if (extra_cost < 0.001) extra_cost = 0.001;
                        mylife = God::instance()->my_energy[ra_addr_]/(extra_cost*60);
                        if (newparentlife < mylife) mylife = newparentlife;
                        else newparentlife = (newparentlife+mylife)/2;
                        statechanged = true;
                        printf("node [%d %f %f] parent [%d %f %f] check [%d %f %f] extra %f %f case 1\n",
                                            ra_addr_, God::instance()->my_lifetime[ra_addr_], mylife,
                                            parentID_, God::instance()->my_lifetime[oldminid],curparentlife,
                                            neiid, God::instance()->my_lifetime[minid],newparentlife,extra,myCost_);
                        if (neiid == 5)
                        {
                            printf("inspect node %d minid %d minrxtr %f mincr %f mineng %f mintr %f, myrate %f\n",neiid,minid,God::instance()->my_rxtr[minid],God::instance()->my_cr[minid],God::instance()->my_energy[minid],God::instance()->my_tr[minid],God::instance()->my_outrate[ra_addr_]);
                        }
                    }
                    else if (newparentlife + 10 < mylife) {
                        newparentlife = God::instance()->my_energy[minid]/(God::instance()->my_cr[minid]+God::instance()->my_outrate[ra_addr_]*God::instance()->my_rxtr[minid]*0.0588);
                        newparentlife = newparentlife/60;

                        if (extra > 0) {
                            extra_cost = God::instance()->my_cr[ra_addr_];
                            amendedextra = God::instance()->my_tr[neiid]-God::instance()->my_rxtr[ra_addr_];
                            extra_cost += God::instance()->my_outrate[ra_addr_]*(amendedextra)*0.0588;
                        } else {
                            if (God::instance()->my_tr[ra_addr_]+amendedextra<0.01) extra_cost = God::instance()->my_cr[minid] + (1-0.01/God::instance()->my_tr[minid])*0.0588;
                            else extra_cost = God::instance()->my_cr[ra_addr_] -
                                              God::instance()->my_tr[ra_addr_]*amendedextra*0.01*0.0588/((God::instance()->my_tr[ra_addr_]+amendedextra)*God::instance()->my_tr[ra_addr_]);
                            amendedextra = God::instance()->my_tr[neiid]-God::instance()->my_rxtr[ra_addr_];
                            extra_cost += God::instance()->my_outrate[ra_addr_]*(amendedextra)*0.0588;
                        }
                        if (extra_cost < 0.001) extra_cost = 0.001;
                        mylife = God::instance()->my_energy[ra_addr_]/(extra_cost*60);
                        if (newparentlife < mylife) mylife = newparentlife;
                        else newparentlife = (newparentlife+mylife)/2;
                        statechanged = true;
                        printf("node [%d %f %f] parent [%d %f %f] check [%d %f %f] extra %f %f case 2\n",
                                            ra_addr_, God::instance()->my_lifetime[ra_addr_], mylife,
                                            parentID_, God::instance()->my_lifetime[oldminid],curparentlife,
                                            neiid, God::instance()->my_lifetime[minid],newparentlife,extra,myCost_);
                        if (neiid == 5)
                        {
                            printf("inspect node %d minid %d minrxtr %f mincr %f mineng %f mintr %f, myrate %f\n",neiid,minid,God::instance()->my_rxtr[minid],God::instance()->my_cr[minid],God::instance()->my_energy[minid],God::instance()->my_tr[minid],God::instance()->my_outrate[ra_addr_]);
                        }
                    }
                    else {
                        statechanged = false;
                    }
                    double min = 0;
                    if (statechanged) {
                        if (curparentlife > mylife) min = mylife;
                        else min = curparentlife;
                        if (newparentlife < min) min = newparentlife;
                        if (min > myCost_+5) {
                            myCost_ = min;
                            statechanged = true;
                        }
                        else {
                            statechanged = false;
                        }
                    }
                    #if 0
                    if(!statechanged){
                        if (minid == God::instance()->min_upstream_id[*nextid]) {
                            if (
                                extra > 0
                                && God::instance()->my_lifetime[*nextid]+10 < God::instance()->my_lifetime[neiid]
                                && God::instance()->my_rxtr[ra_addr_] > God::instance()->my_tr[neiid]
                            ) {
                                statechanged = true;
                                printf("node %d change parent from %d to %d due to local energy compare\n",ra_addr_,*nextid,neiid);
                            }
                        }
                    }
                    #endif
                    if (statechanged) {
                        *nextid = neiid;
                        sinkID_ = 0;
                        *sinkID = sinkID_;
                        God::instance()->current_parent[ra_addr_] = neiid;
                    }
                }
            }
        }
    }
    if (*nextid!=parentID_) {
        printf("node [%d %f] switch from [%d %f %f] to [%d %f %f] @%f\n",
        ra_addr_,tempcost,
        parentID_,God::instance()->min_upstream_life[parentID_],God::instance()->max_upstream_delay[parentID_]+ God::instance()->max_downstream_delay[ra_addr_]+God::instance()->my_tr[parentID_],
        *nextid,God::instance()->min_upstream_life[*nextid],God::instance()->max_upstream_delay[*nextid] + God::instance()->max_downstream_delay[ra_addr_]+God::instance()->my_tr[*nextid],NOW);
        printf("benefit %f -> %f\n",tempcost,myCost_);
    }
    else {printf("node %d no change [parent %d rxtr %f rxdelay %f ]\n",ra_addr_,parentID_,God::instance()->my_rxtr[ra_addr_],God::instance()->max_upstream_delay[ra_addr_]);}
    return myCost_;
}

double ChargingRT::getJROS(int* sinkID, int* nextid) {
    int neiid = 0;
    *sinkID = 0; *nextid = parentID_;
    if (parentID_ <= 0) {
        *nextid = ntbl[ra_addr_][0];
                sinkID_ = 0;
                *sinkID = sinkID_;
        God::instance()->current_parent[ra_addr_] = ntbl[ra_addr_][0];
        printf("node %d defaultparent %d @ %f\n",ra_addr_,ntbl[ra_addr_][0],NOW);
        return myCost_;
    }
    if (lastUpdate_ + routinginterval_ > NOW && parentID_ >= 0 && !God::instance()->my_disconnection[parentID_]) return myCost_;
    lastUpdate_ = NOW;

    if (God::instance()->my_energy[ra_addr_] <= 0.0001) {
        //printf("node %d energytoolow @ %f\n",ra_addr_,NOW);
        return myCost_;
    }

    double mydistance = pow(node_xp[ra_addr_],2)+pow(node_yp[ra_addr_],2);

    double neidistance = 0;
    double minlife = 0;
    int minlifeNodeId = 0;

    std::vector<int> parentset;
    std::multimap< double, int > criticalset;
    std::multimap< double, int > positiveset;
    std::multimap< double, int > negativeset;

    for (int i=0; i< maxneighbor; i++) {
        neiid = ntbl[ra_addr_][i];
        neidistance = pow(node_xp[neiid],2)+pow(node_yp[neiid],2);
        minlife = 0;
        if (neiid != -1 && neidistance+0.0001 < mydistance && neiid<God::instance()->nodes() && !God::instance()->my_disconnection[neiid]) {
            if (God::instance()->isSink(neiid)) {
                myCost_ = God::instance()->my_energy[neiid];
                *nextid = neiid;
                sinkID_ = 0;
                *sinkID = sinkID_;
                God::instance()->current_parent[ra_addr_] = neiid;
                printf("node %d sinkasparent @ %f\n",ra_addr_,NOW);
                return myCost_;
            }
            else {
                minlifeNodeId = God::instance()->min_upstream_id[neiid];
                if (
                    God::instance()->my_totalareaalive[God::instance()->my_areaid[minlifeNodeId]] - God::instance()->my_children_inarea[minlifeNodeId] < ceil(God::instance()->my_totalareareq[minlifeNodeId])+1
                    //|| God::instance()->my_children_inarea[God::instance()->my_subareaid[minlifeNodeId]] - God::instance()->my_children_inarea[minlifeNodeId] < ceil(God::instance()->my_target[minlifeNodeId])
                    )
                {
                    criticalset.insert(std::pair<double,int>(God::instance()->my_lifetime[minlifeNodeId], neiid));
                    printf("node %d add parent node %d minid %d to critical residual %f waste %f life %f %d %d %f %d@ %f\n",
                    ra_addr_,neiid, minlifeNodeId,God::instance()->my_residual[minlifeNodeId], God::instance()->my_subtreeWaste[minlifeNodeId], God::instance()->my_lifetime[minlifeNodeId],
God::instance()->my_totalareaalive[God::instance()->my_areaid[minlifeNodeId]],God::instance()->my_children_inarea[minlifeNodeId],ceil(God::instance()->my_totalareareq[minlifeNodeId]),God::instance()->my_areaid[minlifeNodeId],
NOW);
                }
                else {
                    if (God::instance()->my_residual[minlifeNodeId] > 0.0001) {
                        positiveset.insert(std::pair<double,int>(God::instance()->my_residual[minlifeNodeId], neiid));
                        printf("node %d add parent node %d minid %d to positive residual %f waste %f life %f @ %f\n",
                    ra_addr_,neiid, minlifeNodeId,God::instance()->my_residual[minlifeNodeId], God::instance()->my_subtreeWaste[minlifeNodeId], God::instance()->my_lifetime[minlifeNodeId], NOW);
                    }
                    else {
                        negativeset.insert(std::pair<double,int>(God::instance()->my_subtreeWaste[minlifeNodeId], neiid));
                        printf("node %d add parent node %d minid %d to negative residual %f waste %f life %f @ %f\n",
                    ra_addr_,neiid, minlifeNodeId,God::instance()->my_residual[minlifeNodeId], God::instance()->my_subtreeWaste[minlifeNodeId], God::instance()->my_lifetime[minlifeNodeId], NOW);
                    }
                }
            }
        }
        else {
            if (neiid != -1 && neiid<God::instance()->nodes()) {
            printf("node %d skip %d dist %f %f conn %d @ %f\n",ra_addr_,neiid, neidistance,mydistance, God::instance()->my_disconnection[neiid],NOW);}
        }
    }
    for (std::multimap<double, int>::reverse_iterator it = positiveset.rbegin(); it != positiveset.rend(); ++it) {
        parentset.push_back(it->second);
    }
    for (std::multimap<double, int>::iterator it = negativeset.begin(); it != negativeset.end(); ++it) {
        parentset.push_back(it->second);
    }
    for (std::multimap<double, int>::reverse_iterator it = criticalset.rbegin(); it != criticalset.rend(); ++it) {
        parentset.push_back(it->second);
    }

    int finalselsection = parentID_;
    if (!parentset.empty()) finalselsection = parentset[0];
    if (finalselsection>=0 && God::instance()->min_upstream_id[finalselsection] == parentID_) finalselsection = parentID_;
    *nextid = finalselsection;
    sinkID_ = 0;
    God::instance()->current_parent[ra_addr_] = finalselsection;
    printf("node %d final selection %d @ %f\n",ra_addr_,finalselsection,NOW);
    return myCost_;
}



int ChargingRT::getNextHop(int destid, int* sinkID) {
    int nexthop = -1;
	if (destid == ra_addr_) return -1;
	if (God::instance()->isSink(ra_addr_)) {
        *sinkID = ra_addr_;
        return ra_addr_;
    }
    if (routingType_ == RT_MINCOST) {
        getMinCost(sinkID, &nexthop);
    }
    else if (routingType_ == RT_MINDIST) {
        getMinDist(sinkID, &nexthop);
    }
    else if (routingType_ == RT_LINE) {
        nexthop = ra_addr_-1;
        God::instance()->current_parent[ra_addr_] = nexthop;
    }
    else if (routingType_ == RT_STAR) {
        nexthop = 0;
        God::instance()->current_parent[ra_addr_] = nexthop;
    }
    else if (routingType_ == RT_FIXED) {
        nexthop = parent_id[ra_addr_];
        God::instance()->current_parent[ra_addr_] = nexthop;
    }
    else if (routingType_ == RT_FLOW) {
		//flow
        //nexthop = flowparent_id[ih->saddr()][ch->num_forwards()];
        //*sinkID = flowsink_id[ih->saddr()];
    }
    else if (routingType_ == RT_EA) {
        getEA(sinkID, &nexthop);
    }
    else if (routingType_ == RT_EAPLUS) {
        getEAPlus(sinkID, &nexthop);
    }
    else if (routingType_ == RT_BTREE) {
        nexthop = (ra_addr_-1)/2;
    }
    else if (routingType_ == RT_JROS) {
        getJROS(sinkID, &nexthop);
    }
    else {
        printf("routingType_ %d\n",routingType_);
        nexthop = -1;
    }
    if (nexthop != -1) {
        if (God::instance()->my_disconnection[nexthop]) {
            //printf("node %d newparent %d is out @ %f\n",ra_addr_,nexthop, NOW);
            nexthop = -1;
        }
    }
    if (nexthop == -1 && God::instance()->netinitialized) {
        if (!God::instance()->my_disconnection[ra_addr_]) {
            God::instance()->my_diedat[ra_addr_]=NOW;
            God::instance()->notifyDisconnection(ra_addr_);
            printf("node %d disconnection, routing, energy %f @ %f\n", ra_addr_, God::instance()->my_energy[ra_addr_],NOW);
        }
        God::instance()->my_disconnection[ra_addr_] = true;
    }
    //printf("node %d nexthop %d\n",ra_addr_, nexthop);
	return nexthop;
}

void ChargingRT::recv(Packet* p, Handler* h) {
	struct hdr_cmn *ch = HDR_CMN(p);
	struct hdr_ip *ih = HDR_IP(p);
	// it's a routing message, not an application message
	if(ch->ptype() == PT_CHARGINGRT) {
		recv_chargingrt_pkt(p);
		Packet::free(p);
		return;
	}
	//  Must be a packet I'm originating...
	if((ih->saddr() == ra_addr_) && (ch->num_forwards() == 0)) {
		ch->size() += IP_HDR_LEN;
		if ( (u_int32_t)ih->daddr() != IP_BROADCAST) {
			ih->ttl_ = 30;
		}
	}
	//  I received a packet that I sent, viewed as a routing loop.
	else if(ih->saddr() == ra_addr_) {
        evictNeighbor(parentID_);
        printf("node %d find a loop with parent %d ih->saddr() == ra_addr_\n",ra_addr_,parentID_);
        parentID_ = -1;
		Packet::free(p);
		return;
	 }
	//  Packet I need to forward...
	 else {
		if(--ih->ttl_ == 0) {
		    Packet::free(p);
		    return;
		}
	 }
     updateNeighborTime(ch->prev_hop_);
     if (parentID_ == ch->prev_hop_ && (!God::instance()->isSink(ra_addr_))) {
        evictNeighbor(parentID_);
        printf("node %d find a loop with parent %d parentID_ == ch->prev_hop_ src %d \n",ra_addr_,parentID_,ih->saddr());
        parentID_ = -1;
		Packet::free(p);
		return;
     }

	 if ( (u_int32_t)ih->daddr() != IP_BROADCAST)
	   forward(1, p, 0.0);
	 else
	   forward(0, p, 0.0);
}

void ChargingRT::forward(int rt, Packet *p, double delay) {
	struct hdr_cmn *ch = HDR_CMN(p);
	struct hdr_ip *ih = HDR_IP(p);

	if(ih->ttl_ == 0) {
		Packet::free(p);
		return;
	}
	// recv a packet to me, either dest to me, or broadcasts
	if ((ih->daddr() == here_.addr_|| ih->daddr() == (nsaddr_t)IP_BROADCAST) &&
			(ch->ptype() != PT_CHARGINGRT) && ch->direction() == hdr_cmn::UP) {
		dmux_->recv(p,0);
		return;
	}
#if 1
	// for snoop purpose, its a forwarding unicast message
	if (ih->daddr() != here_.addr_ && ih->daddr() != (nsaddr_t)IP_BROADCAST &&
		ih->saddr() != here_.addr_ && ch->next_hop_ == here_.addr_&&
		(ch->ptype() != PT_CHARGINGRT) && ch->direction() == hdr_cmn::UP) {
		dmux_->recv(p,0);
	}
#endif
	if (rt) {
        int sinkid = 0;
		int nexthop = getNextHop(ih->daddr(), &sinkid);
		if (nexthop < 0) {
			Packet::free(p);
			return;
		}
        else {
            if (parentID_ != nexthop) {
                printf("node %d change parent from %d to %d @ %f\n", ra_addr_,parentID_,nexthop,NOW);
                God::instance()->notifyChangeParent(ra_addr_, parentID_, nexthop);
            }
            parentID_ = nexthop;
        }
        ih->daddr() = sinkid;
		ch->next_hop_ = nexthop;
		ch->addr_type() = NS_AF_INET;
        // process all snooped packet here
        //identifyCriticalNode(ih->saddr());
	}
	else { // if it is a broadcast packet
		assert(ih->daddr() == (nsaddr_t)IP_BROADCAST);
		ch->addr_type() = NS_AF_NONE;
	}
	ch->direction() = hdr_cmn::DOWN;
	ch->num_forwards()++;
	ch->xmit_failure_ = 0;
	ch->xmit_failure_data_ = 0;
    ch->prev_hop_ = ra_addr_;
    packetSent_++;
    Scheduler::instance().schedule(target_, p, 0.001 * Random::uniform());
}

void ChargingRT::recv_chargingrt_pkt(Packet* p) {
	struct hdr_ip *ih = HDR_IP(p);
	if (ih->daddr() == (nsaddr_t)IP_BROADCAST) {
		if (ih->saddr()!=here_.addr_) {
			PacketData* pp = (PacketData*)p->userdata();
			BeaconMsgT_* bmsg = (BeaconMsgT_*)pp->data();
			//if (bmsg->seq>curBseq && bmsg->time>curBtime) {
                //printf("node %d recv beacon msg from %d seq %d\n", ra_addr_, ih->saddr(), bmsg->seq);
				updateNeightorTable(bmsg->myinfo);
				return;
		}
	}
	return;
}

void ChargingRT::send_chargingrt_pkt() {
	return;
}

void ChargingRT::sendBeacon(){
    static int seq;
    BeaconMsgT_ bmsg;
    PacketData* data = new PacketData(sizeof(BeaconMsgT_));
    bmsg.seq = seq++;
    bmsg.time = NOW;
    bmsg.myinfo.id=mnode->nodeid();
    bmsg.myinfo.x=mnode->X();
    bmsg.myinfo.y=mnode->Y();
    bmsg.myinfo.energy = em()->energy();
    if (God::instance()->isSink(ra_addr_)) {
        myCost_ = 0;
        hopCnt_ = 0;
        sinkID_ = ra_addr_;
        parentID_ = ra_addr_;
    }
    else {
//        if (sinkID_ == 0) {
//                sinkID_ = God::instance()->sink_num();
//        }
        sinkID_ = 0;
    }
    bmsg.myinfo.cost = myCost_;
    bmsg.myinfo.sinkID = sinkID_;
    bmsg.myinfo.hopCnt = hopCnt_;
    bmsg.myinfo.timestamp = NOW;
    bmsg.myinfo.parent = parentID_;
    memcpy(data->data(), &bmsg, sizeof(bmsg));

    Packet *p = allocpkt();
    hdr_cmn *ch = hdr_cmn::access(p);
    ch->size() = sizeof(bmsg);
    ch->timestamp() = (u_int32_t)(NOW);
    ch->ptype() = PT_CHARGINGRT;
    ch->num_forwards() = 0;
    ch->next_hop_ = (nsaddr_t)IP_BROADCAST;
    ch->addr_type() = NS_AF_NONE;
    ch->direction() = hdr_cmn::DOWN;       //important: change the packet's direction
    ch->size() += IP_HDR_LEN;
    ch->xmit_failure_ = 0;
    ch->xmit_failure_data_ = 0;

    hdr_ip *iph = hdr_ip::access(p);
    iph->ttl() = 100;
    iph->saddr() = here_.addr_;
    iph->daddr() = IP_BROADCAST;
    iph->sport() = RT_PORT;
    iph->dport() = RT_PORT;
    p->setdata(data);
    packetSent_++;
    Scheduler::instance().schedule(target_, p, CHARGINGJITTER);
    btimer->resched(beaconInterval_+CHARGINGJITTER);
}


void RTBeaconTimer::expire(Event* e) {
    //rtinstance_->sendBeacon();
    int sinkid = 0;
    int nexthop = rtinstance_->getNextHop(0, &sinkid);
    if (nexthop < 0) {
        return;
    }
    else {
        if (rtinstance_->parentID_ != nexthop) {
            printf("node %d change parent from %d to %d @ %f\n", rtinstance_->ra_addr_,rtinstance_->parentID_,nexthop,NOW);
            God::instance()->notifyChangeParent(rtinstance_->ra_addr_, rtinstance_->parentID_, nexthop);
        }
        rtinstance_->parentID_ = nexthop;
    }
    rtinstance_->btimer->resched(1.0+rtinstance_->routinginterval_);
}



