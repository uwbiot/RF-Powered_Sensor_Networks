
/* -*-	Mode:C++; c-basic-offset:8; tab-width:8; indent-tabs-mode:t -*- */
/*
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
 */
/* Ported from CMU/Monarch's code, nov'98 -Padma.*/
/* -*- c++ -*-
   god.h

   General Operations Director

   perform operations requiring omnipotence in the simulation
   */

#ifndef __god_h
#define __god_h

#include <stdarg.h>
#include "bi-connector.h"
#include "object.h"
#include "packet.h"
#include "trace.h"

#include "node.h"
#include "diffusion/hash_table.h"

//yanjun 07
////#include <yjrouteflip.h>
#ifdef SMAC_DOMINANT_SET
#include <vector>
#include <set>
#include <queue>
#endif

#include <udp.h>
// added by yang 09
#include "ctrace.h"
#include "time.h"
#include "parent.h"
// Added by Chalermek  12/1/99

#define MIN_HOPS(i,j)    min_hops[i*num_nodes+j]
#define NEXT_HOP(i,j)    next_hop[i*num_nodes+j]
#define SRC_TAB(i,j)     source_table[i*num_nodes+j]
#define SK_TAB(i,j)      sink_table[i*num_nodes+j]
#define	UNREACHABLE	 0x00ffffff
#define RANGE            250.0                 // trasmitter range in meters

//yanjun 07
#ifdef GOD_IDLE_TIME
#define MIN_METRIC(i,j)	(min_metric[(i)*num_nodes+(j)])
#define UNIDIRECTIONAL_METRIC(i,j)	(unidirectional_metric[(i)*num_nodes+(j)])
#endif


class NodeStatus {
public:
  bool is_source_;
  bool is_sink_;
  bool is_on_trees_;

  NodeStatus() { is_source_ = is_sink_ = is_on_trees_ = false; }
};


// Cut and Paste from setdest.h   -- Chalermek 12/1/99
#if 0
class vector {
public:
	Godvector(double x = 0.0, double y = 0.0, double z = 0.0) {
		X = x; Y = y; Z = z;
	}
	double length() {
		return sqrt(X*X + Y*Y + Z*Z);
	}

	inline void operator=(const Godvector a) {
		X = a.X;
		Y = a.Y;
		Z = a.Z;
	}
	inline void operator+=(const Godvector a) {
		X += a.X;
		Y += a.Y;
		Z += a.Z;
	}
	inline int operator==(const Godvector a) {
		return (X == a.X && Y == a.Y && Z == a.Z);
	}
	inline int operator!=(const Godvector a) {
		return (X != a.X || Y != a.Y || Z != a.Z);
	}
	inline Godvector operator-(const Godvector a) {
		return Godvector(X-a.X, Y-a.Y, Z-a.Z);
	}
	friend inline Godvector operator*(const double a, const Godvector b) {
		return Godvector(a*b.X, a*b.Y, a*b.Z);
	}
	friend inline Godvector operator/(const Godvector a, const double b) {
		return Godvector(a.X/b, a.Y/b, a.Z/b);
	}

	double X;
	double Y;
	double Z;
};
#endif
// ------------------------
#ifdef SMAC_DOMINANT_SET

#define DISTANCE(x1,y1, x2, y2) (sqrt(((x1)-(x2))*((x1)-(x2)) + ((y1)-(y2))*((y1)-(y2))))

struct CDSNeighbor{
	int id;
	int level;
	int height;
};

class YJNode{
public:
    YJNode(){
        reset();
    }
    ~YJNode(){
        reset();
    }
    void reset(){
        id = -1;
        level = -1;
        isPrime = false;
		parentid = -1;
		level = -1;
		height = 0;
		bcastTreeParent = -1;

		isVisited = false;
		isClassified = false;

        x = y = -1.;
        n_list.clear();
        //n_child.clear();
    }
    double x, y; // location
	std::vector<int> n_list;
	std::set<int> cds_neighbors;
	std::set<int> n_child;

    int id;
    int level;
	int height;
    bool isPrime;
	bool isVisited;
	bool isClassified;
	int parentid; // parent node id for BFS
	int bcastTreeParent; // parent node id for Bcast Tree
};
#endif
// ------------------------


class God : public BiConnector {
public:
        God();

        int             command(int argc, const char* const* argv);

        void            recv(Packet *p, Handler *h);
        void            stampPacket(Packet *p);

        int initialized() {
                return num_nodes && min_hops && uptarget_;
        }

        int             hops(int i, int j);
        static God*     instance() { assert(instance_); return instance_; }
        int nodes() { return num_nodes; }

        inline void getGrid(double *x, double *y, double *z) {
		*x = maxX; *y = maxY; *z = gridsize_;
	}

	double total_sleep_time_; 
	double total_active_time_;
	double *duty_cycle_active_; // array which holds total active time for each node
	double *duty_cycle_sleep_; // array which holds total sleep time for each node
	int dropped_packets_by_failed_retransmission_;  // used in xmitFailed in dsragent.cc

#ifdef SMAC_DOMINANT_SET
	bool isDominant(int nodeid);	
	bool getCDSNeighbors(int nodeid, std::vector<struct CDSNeighbor> &nlist);
	void computeNeighbors(int node_num, double radius, YJNode** nodes);
	void dumpBcastTree(YJNode **nodes, int nodes_num);
	void dumpCDSForMatlab(YJNode **nodes, int nodes_num);
	void dumpBFS(YJNode **nodes, int nodes_num);
	int BFS(YJNode **nodes, int start, int node_num) ;
	void getPrimeSet(int node_num, YJNode** nodes, std::vector< std::set<int> > &levelSets);
	void dumpSet(std::set<int> &target, YJNode **nodes);
	void computeCDSTree(int node_num, MobileNode **mb_node);
	YJNode **yjnodes;
#endif

	UdpAgent** sensor_tra_generator_;
	void generateTrafficFromEvent(double x, double y, double radius, int pktsize);
	bool random_event_flag_;


#ifdef GOD_IDLE_TIME
		int getNextHop(int src, int dst);
		void setHopMetric(int src, int dst, double metric);
		void setNodeMetric(int src, double metric); // to all neighbors
		bool need_update_;
		bool need_update_connectivity_;
		bool static_network_;
#endif

#ifdef TWO_FLOW_PDR
        unsigned long total_trans_pkts;
        unsigned long total_success_pkts;
#endif

  // Added by Chalermek 12/1/99

        int  data_pkt_size;        // in bytes. 
        int  num_alive_node;
        int  num_connect;
        int  num_recv;
        int  num_compute;          // number of route-computation times
        double prev_time;          // the previous time it computes the route
        int  num_data_types;
        int  **source_table;
        int  *sink_table;
        int  *num_send;            // for each data type
        Data_Hash_Table dtab;

        void DumpNodeStatus();
        void DumpNumSend();
        void CountNewData(int *attr);
        void IncrRecv();
        bool ExistSource();
        bool ExistSink();
        bool IsPartition();
        void StopSimulation();
        void CountConnect();
        void CountAliveNode();
        void ComputeRoute();      
        int  NextHop(int from, int to);
        void ComputeNextHop();     // Look at min_hops to fill in next_hop
        void Dump();               // Dump all internal data
        bool IsReachable(int i, int j);  // Is node i reachable to node j ?
        bool IsNeighbor(int i, int j);   // Is node i a neighbor of node j ?
        void ComputeW();           // Initialize the connectivity metrix
        void floyd_warshall();     // Calculate the shortest path

        void AddSink(int dt, int skid);
        void AddSource(int dt, int srcid);
        void Fill_for_Sink(int dt, int srcid);
        void Fill_for_Source(int dt, int skid);
        void Rewrite_OIF_Map();
        void UpdateNodeStatus();
        
        // Return number of next oifs in ret_num_oif.
        // Return array of next oifs as return value of the function.

        int *NextOIFs(int dt, int srcid, int curid, int *ret_num_oif);
  
        // serve for GAF algorithm
  
        int load_grid(int,int,int);

        int getMyGrid(double x, double y);
        int getMyLeftGrid(double x, double y);
        int getMyRightGrid(double x, double y);
        int getMyTopGrid(double x, double y);
        int getMyBottomGrid(double x, double y);
        
        inline int getMyGridSize() {
		return gridsize_;
	}

  // -----------------------
        // Yang- 04/25/2010
        CTRACE*	ctrace() { assert(ctrace_); return ctrace_; }
        void addDutyOnTime(int nodeid, double time);
        void logDutyOntime();
        MobileNode* getNode(int targetID){return mb_node[targetID];}
        inline bool isSink(int nodeid){return sink_id[nodeid];}
        inline bool isSource(int nodeid){return source_id[nodeid];}
        void notifyDisconnection(int nodeid);
        void notifyChangeParent(int nodeid, int oldparent, int newparent);
        void reduceSensingCost(int nodeid, double cost);
        double* lastUpdateTime;
        double* lastUpdateEng;
        double* initialEng;
        double* min_upstream_life;
        double* max_upstream_delay;
        double* max_downstream_delay;
        double* my_lifetime;
        double* my_energy;
        double* my_tr;
        double* my_outrate;
        double* my_rxtr;
        double* my_cr;
        int*    current_parent;
        int*    min_upstream_id;
        int*    current_hop;
        // for sensing duty cycle
        int     my_totalnodes;      // total number of nodes in the network, including sink
        int*    my_areaid;          // sensing area
        int*    my_subareaid;       // sensing subarea in the same area
        int*    my_children_inarea;   // total number of children in the subree within the same area (including myself)
        double* my_childrensensing_inarea;    // total sensing allocation of all children in the subtree within the same area (including myself)
        double* my_currentsensing;  // current sensing allocation
        double* my_diedat;          // time when disconnected or out of energy
        bool*   my_disconnection;   // whether I'm disconnected from the network
        double* my_totalareareq;    // total sensing requirement in my area
        int*    my_totalaraenode;   // total number of nodes in my area
        int*    my_totalareaalive;
        double* my_target;          // sensing target for the subtree, including myself
        double* my_subtreelowestlife; // the lowest nodal lifetime estimated by affordable sensing activities
        double* my_subtreeEnergy;
        double* my_subtreeCost;
        double* my_subtreeWaste;
        double* my_residual;
        int* my_subtreeNode;
        int* my_totaltx;
        int* my_totalrx;
        double* my_totalsx;
        bool*   my_rescheduleTarget;
        bool netinitialized;
/*
        double getPrr(int nodeid, int parent);
        void setPrr(int nodeid, int parent, double prr);
        bool getAllowAnycast(int nodeid, int parent);
        void setAllowAnycast(int nodeid, int parent, bool prr);
        double getEEP(int nodeid, double value=-1);
        double getEDC(int nodeid, double value=-1);
        double getETX(int nodeid, double value=-1);
*/
        double* my_etx;
        double* my_edc;
        double* my_eep;
        // -Yang

private:
        int num_nodes;
        int* min_hops;   // square array of num_nodesXnum_nodes
                         // min_hops[i * num_nodes + j] giving 
			 // minhops between i and j
        static God*     instance_;
		
#ifdef GOD_IDLE_TIME
        double* min_metric;   
        double* unidirectional_metric;   
#endif


        // Added by Chalermek    12/1/99

        bool active;
        bool allowTostop;
        MobileNode **mb_node; // mb_node[i] giving pointer to object 
                              // mobile node i
        NodeStatus *node_status;
        int *next_hop;        // next_hop[i * num_nodes + j] giving
                              //   the next hop of i where i wants to send
                              //	 a packet to j.

        int maxX;          // keeping grid demension info: max X, max Y and 
        int maxY;          // grid size
        int gridsize_;
        int gridX;
        int gridY;
        // added by Yang 04/25/2010
        CTRACE* ctrace_;
        double* dutyontime_;
        bool* sink_id;
        bool* source_id;
};

#endif

