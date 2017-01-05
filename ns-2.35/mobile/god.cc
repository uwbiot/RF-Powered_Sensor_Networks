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
 *    notice, this list of conditions and the following disclaim
er in the
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
 * $Header: /cvsroot/nsnam/ns-2/mobile/god.cc,v 1.20 2006/12/27 14:57:23 tom_henderson Exp $
 */

/* Ported from CMU/Monarch's code, nov'98 -Padma.*/

/*
 * god.cc
 *
 * General Operations Director
 *
 * perform operations requiring omnipotence in the simulation
 *
 * NOTE: Tcl node indexs are 0 based, NS C++ node IP addresses (and the
 * node->index() are 1 based.
 *
 */

#include <object.h>
#include <packet.h>
#include <ip.h>
#include <god.h>
#include <sys/param.h>  /* for MIN/MAX */

#include "diffusion/hash_table.h"
#include "mobilenode.h"


God* God::instance_;

static class GodClass : public TclClass {
public:
        GodClass() : TclClass("God") {}
        TclObject* create(int, const char*const*) {
                return (new God);
        }
} class_God;


God::God()
{
    min_hops = 0;
    num_nodes = 0;

    data_pkt_size = 64;
	mb_node = 0;
	next_hop = 0;
	prev_time = -1.0;
	num_alive_node = 0;
	num_connect = 0;
	num_recv = 0;
	num_compute = 0;
	num_data_types = 0;
	source_table = 0;
	sink_table = 0;
	num_send = 0;
	active = false;
	allowTostop = false;
	ctrace_ = 0;
	dutyontime_ = 0;
	sink_id = 0;
	source_id = 0;
    min_upstream_life = 0;
    max_upstream_delay = 0;
    max_downstream_delay = 0;
    netinitialized = false;

	total_sleep_time_ = total_active_time_ = 0.;
	dropped_packets_by_failed_retransmission_ = 0;
#ifdef GOD_IDLE_TIME
	static_network_ = true;
	need_update_ = true;
	need_update_connectivity_ = true;
	min_metric = 0;
	unidirectional_metric = 0;
#endif
#ifdef TWO_FLOW_PDR
	total_trans_pkts = 0;
	total_success_pkts = 0;
#endif
#ifdef RANDOM_RADIUS_EVENTS
	random_event_flag_ = false;
#endif
}


#ifdef GOD_IDLE_TIME
int God::getNextHop(int src, int dst){
	if (active == false) {
		perror("God is off");
		exit(-1);
	}

	if( need_update_ ){
		ComputeRoute();
		need_update_ = false;
	}
	int nexthop = NextHop(src,dst);
	if( nexthop == UNREACHABLE ){
		printf("God::getNextHop: unreachable (%d, %d) uni %f, (%d, %d) uni %f, bi %f %f\n", src, dst, UNIDIRECTIONAL_METRIC(src,dst), dst, src , UNIDIRECTIONAL_METRIC(dst,src), MIN_METRIC(src,dst), MIN_METRIC(dst,src));
	}
	return nexthop;
}

void God::setHopMetric(int src, int dst, double metric){
	if( active == false ){
	  perror("God::setHopMetric not implemented yet");
	  exit(-1);
	}

	if( static_network_ && need_update_connectivity_ ){
		// update connectivity and min_hops info
		floyd_warshall();
	}

	if( UNIDIRECTIONAL_METRIC(src, dst) != metric){
		need_update_ = true;
		UNIDIRECTIONAL_METRIC(src, dst) = metric;
		if( metric == 0 || UNIDIRECTIONAL_METRIC(dst,src) == 0){
			MIN_METRIC(src, dst) = MIN_METRIC(dst, src) = WORSTMETRIC;
		} else {
			double new_cost = 1./(UNIDIRECTIONAL_METRIC(src,dst)*UNIDIRECTIONAL_METRIC(dst,src));
			if( new_cost > WORSTMETRIC )
				new_cost = WORSTMETRIC;

			MIN_METRIC(src,dst) = MIN_METRIC(dst,src) = new_cost;
		}
	}
//	printf("Link %d,%d NewMetric %d\n", src, dst, metric);
}

void God::setNodeMetric(int src, double metric){
	if( active == false ){
	  perror("God::setHopMetric not implemented yet");
	  exit(-1);
	}

	if( static_network_ && need_update_connectivity_ ){
		// update connectivity and min_hops info
		floyd_warshall();
	}

	for (int neighbor=0; neighbor<num_nodes; neighbor++){
		// make sure neighbor is a neighboring node
		if ( MIN_HOPS(src, neighbor) != 1) {
		  continue;
		}
		MIN_METRIC(src, neighbor) = metric;
	}
//	printf("Node %d NewMetric %d\n", src, metric);

	need_update_ = true;
}
#endif


// Added by Chalermek 12/1/99

int God::NextHop(int from, int to)
{
  if (active == false) {
    perror("God is off.\n");
    exit(-1);
  }

  if (from >= num_nodes) {
    perror("index from higher than the maximum number of nodes.\n");
    return -1;
  }

  if (to >= num_nodes) {
    perror("index to higher than the maximum number of nodes.\n");
    return -1;
  }

  return NEXT_HOP(from,to);
}


void God::ComputeNextHop()
{
  if (active == false) {
    return;
  }

  int from, to, neighbor;

#ifdef GOD_IDLE_TIME
  for (from=0; from<num_nodes; from++) {
    for (to=0; to<num_nodes; to++) {

      NEXT_HOP(from,to) = UNREACHABLE;

      if (from==to) {
	NEXT_HOP(from,to) = from;     // next hop is itself.
      }

      if (MIN_HOPS(from, to) == UNREACHABLE) {
		continue;
      }

      for (neighbor=0; neighbor<num_nodes; neighbor++){
		  // make sure neighbor is a neighboring node
		if ( MIN_HOPS(from, neighbor) != 1) {
		  continue;
		}

		if ( fabs(MIN_METRIC(from, to) - MIN_METRIC(neighbor,to) - MIN_METRIC(from, neighbor)) < 0.0001 ) {
		//if ( MIN_METRIC(from, to) == (MIN_METRIC(neighbor,to) + MIN_METRIC(from, neighbor)) ) {
		  NEXT_HOP(from, to) = neighbor;
//		  printf("god think %d to %d using neighbor %d : %f + %f\n", from, to, neighbor,MIN_METRIC(neighbor,to),MIN_METRIC(from, neighbor));
		  break;
		}
      }

    }
  }
#else
  for (from=0; from<num_nodes; from++) {
    for (to=0; to<num_nodes; to++) {

      NEXT_HOP(from,to) = UNREACHABLE;

      if (from==to) {
		NEXT_HOP(from,to) = from;     // next hop is itself.
      }

      if (MIN_HOPS(from, to) == UNREACHABLE) {
		continue;
      }

      for (neighbor=0; neighbor<num_nodes; neighbor++){
	if ( MIN_HOPS(from, neighbor) != 1) {
	  continue;
	}

	if ( MIN_HOPS(from, to) == (MIN_HOPS(neighbor,to) +1) ) {
	  NEXT_HOP(from, to) = neighbor;
	  break;
	}
      }

    }
  }
#endif
}


void God::UpdateNodeStatus()
{
  int i,j;
  int count, cur, sk, srcid, dt;

   for (i=0; i<num_data_types; i++) {
     for (j=0; j<num_nodes; j++) {
       if (SRC_TAB(i,j) != NULL) {
	 node_status[j].is_source_ = true;
       }
     }
   }

   for (i=0; i<num_data_types; i++) {
     for (j=0; j<num_nodes; j++) {
       if (SK_TAB(i,j) > 0) {
	 node_status[j].is_sink_ = true;
       }
     }
   }

   for (dt=0; dt < num_data_types; dt++) {
     for (srcid=0; srcid < num_nodes; srcid++) {
       if (SRC_TAB(dt,srcid) == NULL) 
	 continue;
       for (sk = 0; sk < num_nodes; sk++) {
	 if (SK_TAB(dt, sk) == 0)
	   continue;
	 cur = srcid;
	 count = 0;
	 node_status[cur].is_on_trees_ = true;
	 while (cur != sk) {
	   if (NextHop(cur, sk) == UNREACHABLE)
	     break;

	   assert(NextHop(cur,sk) >= 0 && NextHop(cur, sk) < num_nodes);

	   cur = NextHop(cur, sk);      
	   node_status[cur].is_on_trees_ = true;

	   count ++;
	   assert(count < num_nodes);
	 }
       }
     }
   }

   Dump();
   DumpNodeStatus();
}


void God::DumpNodeStatus()
{
  for (int i=0; i < num_nodes; i++) {
    printf("Node %d status (sink %d, source %d, on_tree %d)\n", i, 
	   node_status[i].is_sink_, node_status[i].is_source_, 
	   node_status[i].is_on_trees_);
  }
}

void God::DumpNumSend()
{
#ifdef DEBUG_OUTPUT
  for (int i=0; i < num_data_types; i++) {
    fprintf(stdout, "God: data type %d distinct events %d\n", i, num_send[i]);
  }
#endif
}


void God::Dump()
{
   int i, j, k, l;

   // Dump min_hops array

   fprintf(stdout,"Dump min_hops\n");
   for(i = 0; i < num_nodes; i++) {
      fprintf(stdout, "%2d) ", i);
      for(j = 0; j < num_nodes; j++)
          fprintf(stdout, "%2d ", min_hops[i * num_nodes + j]);
          fprintf(stdout, "\n");
  }

   // How many times the god compute routes ?

   fprintf(stdout, "God computes routes %d times.\n", num_compute);


   // The following information can be found only when god is active.

   if (active == false) {
     return;
   }

   // Dump next_hop array

   fprintf(stdout, "Dump next_hop\n");
   for (i = 0; i < num_nodes; i++) {
     for (j = 0; j < num_nodes; j++) {
       fprintf(stdout,"NextHop(%d,%d):%d\n",i,j,NEXT_HOP(i,j));
     }
   }


   // What is inside SRC_TAB ?

   fprintf(stdout, "Dump SRC_TAB\n");
   for (i=0; i<num_data_types; i++) {
     fprintf(stdout,"%2d) ",i);
     for (j=0; j<num_nodes; j++) {
       fprintf(stdout,"%2d ", SRC_TAB(i,j) ? 1:0);
     }
     fprintf(stdout,"\n");
   }


   // What is inside OIF_MAP ?

   int *oif_map;

   fprintf(stdout, "Dump OIF_MAP\n");
   for (i=0; i<num_data_types; i++) {
     for (j=0; j<num_nodes; j++) {
       if (SRC_TAB(i,j)!=NULL) {
	 oif_map = SRC_TAB(i,j);
	 fprintf(stdout,"(%2d,%2d)\n",i,j);
	 for (k=0; k<num_nodes; k++) {
	   for (l=0; l<num_nodes; l++) {
	     fprintf(stdout,"%2d ", oif_map[k*num_nodes +l]);
	   }
	   fprintf(stdout,"\n");
	 }
       }
     }
   }



   // What is inside SK_TAB ?

   fprintf(stdout, "Dump SK_TAB\n");
   for (i=0; i<num_data_types; i++) {
     fprintf(stdout,"%2d) ",i);
     for (j=0; j<num_nodes; j++) {
       fprintf(stdout,"%2d ", SK_TAB(i,j));
     }
     fprintf(stdout,"\n");
   }

}


void God::AddSink(int dt, int skid)
{
  if (active == false) {
    return;
  }

  assert(num_data_types > 0);
  assert(num_nodes > 0);
  assert(dt >= 0 && dt < num_data_types);
  assert(skid >= 0 && skid < num_nodes);

  if (SK_TAB(dt,skid) == 1)
     return;

  SK_TAB(dt,skid) = 1;
  Fill_for_Source(dt, skid);
}


void God::AddSource(int dt, int srcid)
{
  if (active == false) {
    return;
  }

  assert(num_data_types > 0);
  assert(num_nodes > 0);
  assert(dt >= 0 && dt < num_data_types);
  assert(srcid >= 0 && srcid < num_nodes);

  if (SRC_TAB(dt,srcid) != 0)
      return;

  SRC_TAB(dt,srcid) = new int[num_nodes * num_nodes];
  bzero((char*) SRC_TAB(dt, srcid), sizeof(int) * num_nodes * num_nodes);
  Fill_for_Sink(dt, srcid);
  //  Dump();
}


void God::Fill_for_Sink(int dt, int srcid)
{
  int sk, cur, count;
  int *oif_map = SRC_TAB(dt, srcid);

  assert(oif_map != NULL);

  for (sk = 0; sk < num_nodes; sk++) {
    if (SK_TAB(dt, sk) == 0)
      continue;
    cur = srcid;
    count = 0;
    while (cur != sk) {
      if (NextHop(cur, sk) == UNREACHABLE)
	break;

      assert(NextHop(cur,sk) >= 0 && NextHop(cur, sk) < num_nodes);

      oif_map[cur*num_nodes + NextHop(cur, sk)] = 1;
      cur = NextHop(cur, sk);      
      count ++;
      assert(count < num_nodes);
    }
  }
}


void God::Fill_for_Source(int dt, int skid)
{
  int src, cur, count;
  int *oif_map;

  for (src = 0; src < num_nodes; src++) {
    if (SRC_TAB(dt, src) == 0)
      continue;
   
    oif_map = SRC_TAB(dt, src);
    cur = src;
    count = 0;
    while (cur != skid) {
      if (NextHop(cur, skid) == UNREACHABLE)
	break;

      assert(NextHop(cur,skid) >= 0 && NextHop(cur, skid) < num_nodes);

      oif_map[cur*num_nodes + NextHop(cur, skid)] = 1;
      cur = NextHop(cur, skid);      
      count ++;
      assert(count < num_nodes);
    }

  }
}


void God::Rewrite_OIF_Map()
{
  for (int dt = 0; dt < num_data_types; dt++) {
    for (int src = 0; src < num_nodes; src++) {
      if (SRC_TAB(dt, src) == NULL)
	continue;

      memset(SRC_TAB(dt,src),'\x00', sizeof(int) * num_nodes * num_nodes);
      Fill_for_Sink(dt, src);
    }
  }
}


int *God::NextOIFs(int dt, int srcid, int curid, int *ret_num_oif) 
{

  if (active == false) {
    perror("God is inactive.\n");
    exit(-1);
  }  

  int *oif_map = SRC_TAB(dt, srcid);
  int count=0;
  int i;

  for (i=0; i<num_nodes; i++) {
    if (oif_map[curid*num_nodes +i] == 1)
      count++;
  }

  *ret_num_oif = count;

  if (count == 0)
    return NULL;

  int *next_oifs = new int[count];
  int j=0;
  
  for (i=0; i<num_nodes; i++) {
    if (oif_map[curid*num_nodes +i] == 1) {
      next_oifs[j] = i;
      j++;    
    }
  }

  return next_oifs;
}



bool God::IsReachable(int i, int j)
{

//  if (MIN_HOPS(i,j) < UNREACHABLE && MIN_HOPS(i,j) >= 0) 
  if (NextHop(i,j) != UNREACHABLE)
     return true;
  else
     return false;
}


bool God::IsNeighbor(int i, int j)
{
  assert(i<num_nodes && j<num_nodes);

#ifndef GOD_IDLE_TIME
  //printf("i=%d, j=%d\n", i,j);
  if (mb_node[i]->energy_model()->node_on() == false ||
      mb_node[j]->energy_model()->node_on() == false ||
      mb_node[i]->energy_model()->energy() <= 0.0 ||
      mb_node[j]->energy_model()->energy() <= 0.0 ) {
    return false;
  }
  return false;
#endif
#if 0
  vector a(mb_node[i]->X(), mb_node[i]->Y(), mb_node[i]->Z());
  vector b(mb_node[j]->X(), mb_node[j]->Y(), mb_node[j]->Z());
  vector d = a - b;

  if (d.length() < RANGE)
    return true;
  else
    return false;  
#endif
    return false;
}


void God::CountConnect()
{
  int i,j;

  num_connect = 0;

  for (i=0; i<num_nodes; i++) {
    for (j=i+1; j<num_nodes; j++) {
      if (MIN_HOPS(i,j) != UNREACHABLE) {
	num_connect++;
      }
    }
  }
}


void God::CountAliveNode()
{
  int i;

  num_alive_node = 0;

  for (i=0; i<num_nodes; i++) {
    if (mb_node[i]->energy_model()->energy() > 0.0) {
      num_alive_node++;
    }
  }

}


bool God::ExistSource()
{
  int dtype, i;

  for (dtype = 0; dtype < num_data_types; dtype++) {
    for (i=0; i<num_nodes; i++) {
      if (SRC_TAB(dtype, i) != 0)
	return true;
    }
  }

  return false;
}


bool God::ExistSink()
{
  int dtype, i;

  for (dtype = 0; dtype < num_data_types; dtype++) {
    for (i=0; i<num_nodes; i++) {
      if (SK_TAB(dtype, i) != 0)
	return true;
    }
  }

  return false;
}


bool God::IsPartition()
{
  int dtype, i, j, k;
  int *oif_map;

  for (dtype = 0; dtype < num_data_types; dtype ++) {
    for (i = 0; i < num_nodes; i++) {
      if (SRC_TAB(dtype,i) == NULL)
	continue;
      oif_map = SRC_TAB(dtype, i);
      for (j = 0; j < num_nodes; j++) {
	for (k = 0; k < num_nodes; k++) {
	  if (oif_map[j*num_nodes + k] != 0)
	    return false;
	}
      }
    }
  }

  return true;
}


void God::ComputeRoute() 
{
  if (active == false) {
    return;
  }

  // get connectivity and min_hop info 
  floyd_warshall();

//yanjun 07 skip the rest
#ifdef GOD_IDLE_TIME
  // update the new metric
//  printf("God Updating metric at %f\n", Scheduler::instance().clock());

  // reset the metrics for all non-single hop routes
  for(int i = 0; i < num_nodes; i++) {
     for(int j = 0; j < num_nodes; j++) {
		 if( i == j ){
			 MIN_HOPS(i,j) = 0;
		 }
		 else if(MIN_HOPS(i,j) != 1){
			MIN_METRIC(i,j) = UNKNOWNMETRIC;
		 }
	 }
  }

  for(int i = 0; i < num_nodes; i++) {
     for(int j = 0; j < num_nodes; j++) {
		 for(int k = 0; k < num_nodes; k++) {
			MIN_METRIC(j,k) = MIN(MIN_METRIC(j,k), MIN_METRIC(j,i) + MIN_METRIC(i,k));
		//	printf("%d, %d min_metric is %f\n", j,k, MIN_METRIC(j,k));
		 }
     }
  }
#endif

  ComputeNextHop();

//yanjun 07 skip the rest
#ifndef GOD_IDLE_TIME
  Rewrite_OIF_Map();
  CountConnect();
  CountAliveNode();
#endif

  prev_time = NOW;
  num_compute++;

//yanjun 07 skip the rest
#ifdef GOD_IDLE_TIME
  return;
#endif

  if (allowTostop == false)
    return;

  if ( ExistSink() == true && ExistSource() == true && IsPartition() == true)
    StopSimulation();
}


void God::CountNewData(int *attr)
{
	/* disable diffusion temporarily 
  if (dtab.GetHash(attr) == NULL) {
    num_send[attr[0]]++;
    dtab.PutInHash(attr);
  }
  */
}


void God::IncrRecv()
{
  num_recv++;

  //  printf("God: num_connect %d, num_alive_node %d at recv pkt %d\n",
  // num_connect, num_alive_node, num_recv);
}


void God::StopSimulation() 
{
  Tcl& tcl=Tcl::instance();

  printf("Network parition !! Exiting... at time %f\n", NOW);
  tcl.evalf("[Simulator instance] at %lf \"finish\"", (NOW)+0.000001);
  tcl.evalf("[Simulator instance] at %lf \"[Simulator instance] halt\"", (NOW)+0.000002);
}


// Modified from setdest.cc -- Chalermek 12/1/99

void God::ComputeW()
{
  int i, j;
  int *W = min_hops;

  memset(W, '\xff', sizeof(int) * num_nodes * num_nodes);

  for(i = 0; i < num_nodes; i++) {
     W[i*num_nodes + i] = 0;     
     for(j = i+1; j < num_nodes; j++) {
	W[i*num_nodes + j] = W[j*num_nodes + i] = 
	                     IsNeighbor(i,j) ? 1 : INFINITY;
//yanjun 07 // initially, set metrics of unknow links to a very large number, mimic shortest-hop routing
#ifdef GOD_IDLE_TIME
	if(W[i*num_nodes + j] == 1){
		MIN_METRIC(i,j) = MIN_METRIC(j,i) = WORSTMETRIC;
	} else {
		MIN_METRIC(i,j) = MIN_METRIC(j,i) = UNKNOWNMETRIC;
	}
#endif
     }
  }
}

void God::floyd_warshall()
{
  int i, j, k;

//yanjun 07
#ifdef GOD_IDLE_TIME
  if( static_network_ && need_update_connectivity_ ){
	  ComputeW();	// the connectivity matrix
	  for(i = 0; i < num_nodes; i++) {
		 for(j = 0; j < num_nodes; j++) {
			 for(k = 0; k < num_nodes; k++) {
				MIN_HOPS(j,k) = MIN(MIN_HOPS(j,k), MIN_HOPS(j,i) + MIN_HOPS(i,k));
			 }
		 }
	  }
		// only call this once in a static network
	  need_update_connectivity_ = false;
  } 
# else
  ComputeW();	// the connectivity matrix

  for(i = 0; i < num_nodes; i++) {
     for(j = 0; j < num_nodes; j++) {
	 for(k = 0; k < num_nodes; k++) {
	    MIN_HOPS(j,k) = MIN(MIN_HOPS(j,k), MIN_HOPS(j,i) + MIN_HOPS(i,k));
	 }
     }
  }
#endif


#ifdef SANITY_CHECKS

  for(i = 0; i < num_nodes; i++)
     for(j = 0; j < num_nodes; j++) {
	assert(MIN_HOPS(i,j) == MIN_HOPS(j,i));
	assert(MIN_HOPS(i,j) <= INFINITY);
     }
#endif

}

// --------------------------


int
God::hops(int i, int j)
{
        return min_hops[i * num_nodes + j];
}


void
God::stampPacket(Packet *p)
{
        hdr_cmn *ch = HDR_CMN(p);
        struct hdr_ip *ih = HDR_IP(p);
        nsaddr_t src = ih->saddr();
        nsaddr_t dst = ih->daddr();

        assert(min_hops);

        if (!packet_info.data_packet(ch->ptype())) return;

        if (dst > num_nodes || src > num_nodes) return; // broadcast pkt
   
        ch->opt_num_forwards() = min_hops[src * num_nodes + dst];
}


void 
God::recv(Packet *, Handler *)
{
        abort();
}

int
God::load_grid(int x, int y, int size)
{
	maxX =  x;
	maxY =  y;
	gridsize_ = size;
	
	// how many gridx in X direction
	gridX = (int)maxX/size;
	if (gridX * size < maxX) gridX ++;
	
	// how many grid in Y direcion
	gridY = (int)maxY/size;
	if (gridY * size < maxY) gridY ++;

	printf("Grid info:%d %d %d (%d %d)\n",maxX,maxY,gridsize_,
	       gridX, gridY);

	return 0;
}
 
// return the grid that I am in
// start from left bottom corner, 
// from left to right, 0, 1, ...

int
God::getMyGrid(double x, double y)
{
	int xloc, yloc;
	
	if (x > maxX || y >maxY) return(-1);
	
	xloc = (int) x/gridsize_;
	yloc = (int) y/gridsize_;
	
	return(yloc*gridX+xloc);
}

int
God::getMyLeftGrid(double x, double y)
{

	int xloc, yloc;
	
	if (x > maxX || y >maxY) return(-1);
	
	xloc = (int) x/gridsize_;
	yloc = (int) y/gridsize_;

	xloc--;
	// no left grid
	if (xloc < 0) return (-2);
	return(yloc*gridX+xloc);
}

int
God::getMyRightGrid(double x, double y)
{

	int xloc, yloc;
	
	if (x > maxX || y >maxY) return(-1);
	
	xloc = (int) x/gridsize_;
	yloc = (int) y/gridsize_;

	xloc++;
	// no left grid
	if (xloc > gridX) return (-2);
	return(yloc*gridX+xloc);
}

int
God::getMyTopGrid(double x, double y)
{

	int xloc, yloc;
	
	if (x > maxX || y >maxY) return(-1);
	
	xloc = (int) x/gridsize_;
	yloc = (int) y/gridsize_;

	yloc++;
	// no top grid
	if (yloc > gridY) return (-2);
	return(yloc*gridX+xloc);
}

int
God::getMyBottomGrid(double x, double y)
{

	int xloc, yloc;
	
	if (x > maxX || y >maxY) return(-1);
	
	xloc = (int) x/gridsize_;
	yloc = (int) y/gridsize_;

	yloc--;
	// no top grid
	if (yloc < 0 ) return (-2);
	return(yloc*gridX+xloc);
}

#ifdef SMAC_DOMINANT_SET
void God::computeNeighbors(int node_num, double radius, YJNode** nodes){
    // build up neighbor list
    double limit = radius * radius;
    double xdiff, ydiff;
    for(int i=1; i<=node_num; i++){
        for( int j=i+1; j<= node_num; j++){
            xdiff = nodes[i]->x - nodes[j]->x;
            ydiff = nodes[i]->y - nodes[j]->y;
            double d_square = xdiff*xdiff + ydiff*ydiff;
            if( d_square < limit ){
                nodes[i]->n_list.push_back(j);
                nodes[j]->n_list.push_back(i);
            }
        }
    }
}

void God::dumpCDSForMatlab(YJNode **nodes, int nodes_num){
	for(int i=1; i <= nodes_num; i++){
		// output location
		fprintf(stdout, "%d\t%f\t%f\t", i, nodes[i]->x, nodes[i]->y);

		// output children
		if( nodes[i]->isPrime ){
			for (std::set<int>::const_iterator ite = nodes[i]->n_child.begin(); ite != nodes[i]->n_child.end(); ite++) {
				fprintf(stdout, "%d ", *ite);
			}
		} else {
			// no child
			fprintf(stdout, "%d ", -1);
		}
			
		fprintf(stdout,"\n");
	}
}
void God::dumpBcastTree(YJNode **nodes, int nodes_num){
	bool foundchild = false;
	for(int i=1; i <= nodes_num; i++){
		// output location
		fprintf(stdout, "%d\t%f\t%f\t", i, nodes[i]->x, nodes[i]->y);

		// output children
		foundchild = false;
        for (std::vector<int>::const_iterator ite = nodes[i]->n_list.begin(); ite != nodes[i]->n_list.end(); ite++) {
			if( nodes[*ite]->bcastTreeParent == i ){
				foundchild = true;
				fprintf(stdout, "%d ", *ite);
			}
		}
		// no child found
		if( foundchild == false ){
			fprintf(stdout, "%d ", -1);
		}
			
		fprintf(stdout,"\n");
	}
}

void God::dumpBFS(YJNode **nodes, int nodes_num){
	bool foundchild;
	for(int i=1; i <= nodes_num; i++){
		// output location
		fprintf(stdout, "%d\t%f\t%f\t", i, nodes[i]->x, nodes[i]->y);

		// output children
		foundchild = false;
        for (std::vector<int>::const_iterator ite = nodes[i]->n_list.begin(); ite != nodes[i]->n_list.end(); ite++) {
			if( nodes[*ite]->parentid == i ){
				foundchild = true;
				fprintf(stdout, "%d ", *ite);
			}
		}
		// no child found
		if( foundchild == false ){
			fprintf(stdout, "%d ", -1);
		}
			
		fprintf(stdout,"\n");
	}
}

/** construct a BFS with start as root, return the level of the tree, 0 for broken BFS */
int God::BFS(YJNode **nodes, int start, int node_num) {
	std::queue<int> next;
	int nodecount = 0;
	int BFSlevel = 0;

    nodes[start]->isVisited = true;
    nodes[start]->level = 0; 		// sink node has level 0
    next.push(start);
	nodecount++;

    while (!next.empty()) {
        int target = next.front();
        next.pop();
        // Here is the point where you can examine the u th vertex of graph
        for (std::vector<int>::const_iterator ite = nodes[target]->n_list.begin(); ite != nodes[target]->n_list.end(); ite++) {
            // Look through neighbors.
            int curneighbor = *ite;
            if (nodes[curneighbor]->isVisited == false) {
                // If v is unvisited.
				nodecount++;
				nodes[curneighbor]->isVisited = true;
				nodes[curneighbor]->parentid = target;
				nodes[curneighbor]->level = nodes[target]->level + 1;
				if( nodes[curneighbor]->level > BFSlevel ){
					BFSlevel = nodes[curneighbor]->level;
				}
                next.push(curneighbor);
            }
        }
    }

	if( nodecount == node_num ){
		return BFSlevel;
	} else {
		return 0;
	}
}

void God::getPrimeSet(int node_num, YJNode** nodes, std::vector< std::set<int> > &levelSets){
	int curindex;
	for( std::vector< std::set<int> >::const_iterator vite = levelSets.begin();
			vite != levelSets.end();
			vite++){
		for( std::set<int>::const_iterator site = vite->begin();
				site != vite->end();
				site++ ){
			curindex = *site;
			if( nodes[curindex]->isClassified == false ){
//				cout << "prime " << curindex << ": ";
				nodes[curindex]->isPrime = true;
				nodes[curindex]->isClassified = true;
				for (std::vector<int>::const_iterator ite = nodes[curindex]->n_list.begin(); ite != nodes[curindex]->n_list.end(); ite++) {
					// all neighbors are classified as secondary nodes
					if( nodes[*ite]->isClassified == false )
//						cout << *ite << " ";
					nodes[*ite]->isClassified = true;
				}
			}
		}
	}
}

void God::dumpSet(std::set<int> &target, YJNode **nodes){
	std::set<int>::const_iterator ite = target.begin();
	fprintf(stdout, "Set includes:\n");
	for(; ite != target.end(); ite++){
		fprintf(stdout, "%d dominants: \n", *ite);
		for (std::vector<int>::const_iterator iten = nodes[*ite]->n_list.begin(); iten != nodes[*ite]->n_list.end(); iten++) {
			fprintf(stdout, "%d ", *iten);
		}
		fprintf(stdout, "\n");
	}
}

bool God::isDominant(int nodeid){
	return yjnodes[nodeid]->isPrime;
}

bool God::getCDSNeighbors(int nodeid, std::vector<struct CDSNeighbor> &nlist){
	for(std::set<int>::const_iterator site = yjnodes[nodeid]->cds_neighbors.begin();
			site != yjnodes[nodeid]->cds_neighbors.end(); site++ ){
		struct CDSNeighbor tmp = {*site, yjnodes[*site]->level, yjnodes[*site]->height};
		nlist.push_back(tmp);
	}
	return yjnodes[nodeid]->isPrime;
}

void God::computeCDSTree(int node_num, MobileNode **mb_node){
    double radius = 250; // max range for BASE rate
    for(int i=1; i<=node_num; i++){
        yjnodes[i]->x = mb_node[i]->X();
        yjnodes[i]->y = mb_node[i]->Y();
        yjnodes[i]->id = i;
		yjnodes[i]->isPrime = false;
    }

    computeNeighbors(node_num, radius, yjnodes);

	int BFSlevel;
    // construct BFS tree, make yjnodes[1] the root
	if( (BFSlevel = BFS(yjnodes, 1, node_num)) ){
		// if a BFS tree rooted at yjnodes[1] reaches all nodes
		//dumpBFS(yjnodes, node_num);
	} else {
		fprintf(stderr, "The BFS fails to reach all nodes\n");
		exit(1);
	}

	//fprintf(stdout, "BFS has max level of %d\n", BFSlevel);

	std::vector< std::set<int> > levelSets(BFSlevel+1);
	for(int i = 1; i <= node_num; i++){
		levelSets[yjnodes[i]->level].insert(i);
	}

	// find prime set
	//set<int> primeset;
	getPrimeSet(node_num, yjnodes, levelSets); // starts with node 1
	//dumpSet(primeset, yjnodes);

	// assign parents, skip depth 0 since it's the root node
	for(int depth = 1; depth <= BFSlevel; depth++){
		// set parent node for all nodes at depth of BFS tree
		for( std::set<int>::const_iterator ite = levelSets[depth].begin();
				ite != levelSets[depth].end();
				ite++){
			if( yjnodes[*ite]->isPrime ){
				// if this node is a prime node
				double tmp_distance, candiadate_distance = 1.1e10;
				int tmp_parent;
				std::vector<int>::const_iterator vite;
				for( vite = yjnodes[*ite]->n_list.begin();
						vite != yjnodes[*ite]->n_list.end();
						vite++){
					if( yjnodes[*vite]->level == depth - 1 ){
						// find a prime parent at level of (depth - 1) 
						tmp_distance = DISTANCE(yjnodes[*vite]->x, yjnodes[*vite]->y, yjnodes[*ite]->x, yjnodes[*ite]->y);
						if( tmp_distance < candiadate_distance ){
							candiadate_distance = tmp_distance;
							tmp_parent = *vite;
						}
					}
				}
				// sanity
				if( candiadate_distance > 1e10 ){
					fprintf(stderr, "node %d cannot find1 a bcastTreeParent \n", *ite);
					exit(1);
				} else {
					yjnodes[*ite]->bcastTreeParent = tmp_parent;
					yjnodes[tmp_parent]->n_child.insert(*ite);
					yjnodes[tmp_parent]->isPrime = true;
				}
			} else {
				// if this node is a secondary node
				int candidate_level = yjnodes[*ite]->level;
				double tmp_distance, candiadate_distance = 1.1e10;
				int tmp_parent;
				std::vector<int>::const_iterator vite;
				for( vite = yjnodes[*ite]->n_list.begin();
						vite != yjnodes[*ite]->n_list.end();
						vite++){
					if( yjnodes[*vite]->isPrime ){
						if( yjnodes[*vite]->level <= candidate_level ){
							tmp_distance = DISTANCE(yjnodes[*vite]->x, yjnodes[*vite]->y, yjnodes[*ite]->x, yjnodes[*ite]->y);
							if( tmp_distance < candiadate_distance ){
								candiadate_distance = tmp_distance;
								candidate_level = yjnodes[*vite]->level;
								tmp_parent = *vite;
							}
						}
					}
				}
				// sanity
				if( candiadate_distance > 1e10 ){
					fprintf(stderr, "node %d cannot find2 a bcastTreeParent\n", *ite);
					exit(1);
				} else {
					yjnodes[*ite]->bcastTreeParent = tmp_parent;
					yjnodes[tmp_parent]->n_child.insert(*ite);
				}
			}
		}
	}

	// prime nodes are those with children
	for(int i=1; i <= node_num; i++){
		if( yjnodes[i]->n_child.size() > 0 )
			yjnodes[i]->isPrime = true;
		else
			yjnodes[i]->isPrime = false;
	}
	
	// try to remove a prime nodes with only one child if the child can find an alternate parent
	int changed;
remove_primes:
	changed = 0;
	for(int i = 1; i <= node_num; i++){
		if( yjnodes[i]->isPrime && yjnodes[i]->n_child.size() == 1 ){
			int child = *(yjnodes[i]->n_child.begin());
			int exist_level = yjnodes[i]->level;
			std::vector<int>::const_iterator vite;
			for( vite = yjnodes[child]->n_list.begin();
					vite != yjnodes[child]->n_list.end();
					vite++){
				// skip current parent
				if( *vite == i ) continue;

				if( yjnodes[*vite]->isPrime && yjnodes[*vite]->level <= exist_level ){
					// unmark current parent
					yjnodes[i]->n_child.erase(child);
					yjnodes[i]->isPrime = false;

					// add child to the children list of the new prime node
					yjnodes[*vite]->n_child.insert(child);
					// update change counter
					changed++;
//					cout << "child " << child << " is moved from parnet " << i << " to parent " << *vite << endl;
				}
			}
		}
	}
	if( changed > 0 ) goto remove_primes;

	dumpCDSForMatlab(yjnodes, node_num);

	// calculate neighborhood info on CDS nodes
	for(int i=1; i <= node_num; i++){
		if( yjnodes[i]->isPrime ){
			for(std::set<int>::const_iterator site = yjnodes[i]->n_child.begin();
					site != yjnodes[i]->n_child.end(); site++){
				if(yjnodes[*site]->isPrime){
					yjnodes[i]->cds_neighbors.insert(*site);
					yjnodes[*site]->cds_neighbors.insert(i);
				}
			}
		}
	}

	// find out the max height of each prime node
	for(int i=2; i<=node_num; i++){
		if( yjnodes[i]->isPrime == false ){
			int cur_child = i;
			int cur_parent = yjnodes[i]->bcastTreeParent;
			int cur_height = 1;
			// traverse up to the root 
			while(true){
				if( yjnodes[cur_parent]->height < cur_height ){
					// if this path raise the height of current parent node
					// recursively update all parent node along the path
					yjnodes[cur_parent]->height = cur_height;

					if( cur_parent == 1 ){
						// already root
						break;
					} else {
						// keep going up
						cur_height++;
						cur_child = cur_parent;
						cur_parent = yjnodes[cur_parent]->bcastTreeParent;
					}
				} else {
					break;
				}
			}
		}
	}
}
#endif

int 
God::command(int argc, const char* const* argv)
{
	Tcl& tcl = Tcl::instance(); 
	if ((instance_ == 0) || (instance_ != this))
        instance_ = this; 

        if (argc == 2) {
    #ifdef SMAC_DOMINANT_SET
            if(strcmp(argv[1], "computeCDSTree") == 0) {
                computeCDSTree(num_nodes-1, mb_node);
                return TCL_OK;
            }
    #endif

    #ifdef TWO_FLOW_PDR
            if(strcmp(argv[1], "overall_pdr") == 0) {
                //printf("total_success_pkts %u total_trans_pkts %u\n", total_success_pkts, total_trans_pkts);
                if( total_success_pkts ){
                    tcl.resultf("%f", 1.*total_success_pkts/total_trans_pkts);
                    return TCL_OK;
                } else {
                    fprintf(stderr, "total_trans_pkts is 0 ");
                    return TCL_ERROR;
                }
            }
    #endif
            if(strcmp(argv[1], "update_node_status") == 0) {
                UpdateNodeStatus();
                return TCL_OK;
            }

            if(strcmp(argv[1], "compute_route") == 0) {
                ComputeRoute();
                return TCL_OK;
            }

            if(strcmp(argv[1], "dump") == 0) {
                Dump();
                return TCL_OK;
            }

            if (strcmp(argv[1], "dump_num_send") == 0) {
                DumpNumSend();
                return TCL_OK;
            }

            if (strcmp(argv[1], "on") == 0) {
                active = true;
                return TCL_OK;
            }

            if (strcmp(argv[1], "off") == 0) {
                active = false;
                return TCL_OK;
            }

            if (strcmp(argv[1], "allow_to_stop") == 0) {
              allowTostop = true;
              return TCL_OK;
            }

            if (strcmp(argv[1], "not_allow_to_stop") == 0) {
              allowTostop = false;
              return TCL_OK;
            }

            if(strcmp(argv[1], "num_nodes") == 0) {
                tcl.resultf("%d", nodes());
                return TCL_OK;
            }

            if (strcmp(argv[1], "log_dutycycle") == 0) {
                logDutyOntime();
                return TCL_OK;
            }
            else if(strcmp(argv[1], "total_sleep_time") == 0) {
                tcl.resultf("%f", total_sleep_time_);
                return TCL_OK;
            } else if(strcmp(argv[1], "total_active_time") == 0) {
                tcl.resultf("%f", total_active_time_);
                return TCL_OK;
            } else if(strcmp(argv[1], "dropped_packets_by_failed_retransmission") == 0) {
                tcl.resultf("%d", dropped_packets_by_failed_retransmission_);
                return TCL_OK;
            }
        }
        else if(argc == 3) {
            if (strcasecmp(argv[1], "add-logfile") == 0) {
                ctrace_ = (CTRACE*)tcl.lookup(argv[2]);
                if (ctrace_!=0) {
                    time_t rawtime;
                    time ( &rawtime );
                    //ctrace()->log("starttime: ns2 %f system %s \n", NOW, ctime(&rawtime));
                    return (TCL_OK);
                }
                else {return TCL_ERROR;}
            }
            else if (strcasecmp(argv[1], "setassink") == 0) {
                int sinknodeid = atoi(argv[2]);
                sink_id[sinknodeid] = true;
                source_id[sinknodeid] = false;
                double nx = node_xp[sinknodeid];
                double ny = node_yp[sinknodeid];
                //mb_node[sinknodeid]->setX(nx);
                //mb_node[sinknodeid]->setY(ny);
                //my_areaid[sinknodeid]=area_id[sinknodeid];
                return TCL_OK;
            }
            else if (strcasecmp(argv[1], "setassource") == 0) {
                int sourcenodeid = atoi(argv[2]);
                sink_id[sourcenodeid] = false;
                source_id[sourcenodeid] = true;
                double nx = node_xp[sourcenodeid];
                double ny = node_yp[sourcenodeid];
                //mb_node[sourcenodeid]->setX(nx);
                //mb_node[sourcenodeid]->setY(ny);
                //my_areaid[sourcenodeid]=area_id[sourcenodeid];
                return TCL_OK;
            }
            else if (strcasecmp(argv[1], "setasrouter") == 0) {
                int sourcenodeid = atoi(argv[2]);
                sink_id[sourcenodeid] = false;
                source_id[sourcenodeid] = false;
                double nx = node_xp[sourcenodeid];
                double ny = node_yp[sourcenodeid];
                //mb_node[sourcenodeid]->setX(nx);
                //mb_node[sourcenodeid]->setY(ny);
                //my_areaid[sourcenodeid]=area_id[sourcenodeid];
                return TCL_OK;
            }
            else if(strcmp(argv[1], "node_total_sleep_time") == 0) {
                int nodeid = atoi(argv[2]);
                if( nodeid <= 0 || nodeid > num_nodes ){
                    fprintf(stderr, "God::command: invalid node id for node_total_sleep_time\n");
                    exit(-1);
                }
                tcl.resultf("%f", duty_cycle_sleep_[nodeid]);
                return TCL_OK;
            } else if(strcmp(argv[1], "node_total_active_time") == 0) {
                int nodeid = atoi(argv[2]);
                if( nodeid <= 0 || nodeid > num_nodes ){
                    fprintf(stderr, "God::command: invalid node id for node_total_active_time\n");
                    exit(-1);
                }
                tcl.resultf("%f", duty_cycle_active_[nodeid]);
                return TCL_OK;
            } else if(strcasecmp(argv[1], "switchRandomEvent") == 0) {
                if( 0 == strcasecmp(argv[2], "ON") ){
                    random_event_flag_ = true;
                } else if( 0 == strcasecmp(argv[2], "OFF") ){
                    random_event_flag_ = false;
                } else {
                    fprintf(stderr, "god::switchRandomEvent got an unexpected parameter\n");
                    return TCL_ERROR;
                }
                return TCL_OK;
            }

            if (strcasecmp(argv[1], "is_source") == 0) {
                int node_id = atoi(argv[2]);

                if (node_status[node_id].is_source_ == true) {
                    tcl.result("1");
                } else {
                    tcl.result("0");
                }
                return TCL_OK;
            }

            if (strcasecmp(argv[1], "is_sink") == 0) {
                int node_id = atoi(argv[2]);

                if (node_status[node_id].is_sink_ == true) {
                    tcl.result("1");
                } else {
                    tcl.result("0");
                }
                return TCL_OK;
            }

            if (strcasecmp(argv[1], "is_on_trees") == 0) {
                int node_id = atoi(argv[2]);

                if (node_status[node_id].is_on_trees_ == true) {
                    tcl.result("1");
                } else {
                    tcl.result("0");
                }
                return TCL_OK;
            }

            if (strcasecmp(argv[1], "num_nodes") == 0) {
                assert(num_nodes == 0);
                // index always starts from 0
                num_nodes = atoi(argv[2]);

                assert(num_nodes > 0);
                printf("num_nodes is set %d\n", num_nodes);
                my_totalnodes = num_nodes;
                min_hops = new int[num_nodes * num_nodes];
                // keep track of duty cycle for eachnode
                duty_cycle_active_ = new double[num_nodes+1];
                duty_cycle_sleep_ = new double[num_nodes+1];
                bzero((char*) duty_cycle_active_, sizeof(double) * (num_nodes + 1));
                bzero((char*) duty_cycle_sleep_, sizeof(double) * (num_nodes + 1));

            #ifdef GOD_IDLE_TIME
                min_metric = new double[num_nodes * num_nodes];
                bzero((char*) min_metric, sizeof(double) * num_nodes * num_nodes);
                unidirectional_metric = new double[num_nodes * num_nodes];
                bzero((char*) unidirectional_metric, sizeof(double) * num_nodes * num_nodes);
            #endif

                #ifdef SMAC_DOMINANT_SET
                // initialize nodes (position)
                yjnodes = new YJNode*[num_nodes];
                for(int i=1; i<=num_nodes; i++){
                    yjnodes[i] = new YJNode;
                    yjnodes[i]->isPrime = true; // if getCDSNeighbors is not called, every node is in CDS
                }
                #endif

                sensor_tra_generator_ = new UdpAgent* [num_nodes];
                bzero( (char*) sensor_tra_generator_, sizeof(UdpAgent *) * num_nodes);

                mb_node = new MobileNode*[num_nodes];
                node_status = new NodeStatus[num_nodes];
                next_hop = new int[num_nodes * num_nodes];
                // Yang- 04/25/2010
                dutyontime_ = new double[num_nodes];
                sink_id = new bool[num_nodes];
                source_id = new bool[num_nodes];
                bzero((char*)sink_id, sizeof(bool)*num_nodes);
                bzero((char*)source_id, sizeof(bool)*num_nodes);
                bzero((char*)dutyontime_, sizeof(double)*num_nodes);
                lastUpdateTime = new double[num_nodes];
                lastUpdateEng = new double[num_nodes];
                initialEng = new double[num_nodes];
                bzero((char*)lastUpdateTime, sizeof(double)*num_nodes);
                bzero((char*)lastUpdateEng, sizeof(double)*num_nodes);
                bzero((char*)initialEng, sizeof(double)*num_nodes);
                min_upstream_life = new double[num_nodes];
                max_upstream_delay = new double[num_nodes];
                max_downstream_delay = new double[num_nodes];
                bzero((char*)min_upstream_life, sizeof(double)*num_nodes);
                bzero((char*)max_upstream_delay, sizeof(double)*num_nodes);
                bzero((char*)max_downstream_delay, sizeof(double)*num_nodes);
                current_parent = new int[num_nodes];
                current_hop = new int[num_nodes];
                min_upstream_id = new int[num_nodes];
                my_lifetime = new double[num_nodes];
                my_energy = new double[num_nodes];
                my_tr = new double[num_nodes];
                my_cr = new double[num_nodes];
                my_outrate = new double[num_nodes];
                my_rxtr = new double[num_nodes];
                bzero((char*)min_upstream_id, sizeof(int)*num_nodes);
                bzero((char*)current_parent, sizeof(int)*num_nodes);
                bzero((char*)current_hop, sizeof(int)*num_nodes);
                bzero((char*)my_lifetime, sizeof(double)*num_nodes);
                bzero((char*)my_energy, sizeof(double)*num_nodes);
                bzero((char*)my_tr, sizeof(double)*num_nodes);
                bzero((char*)my_cr, sizeof(double)*num_nodes);
                bzero((char*)my_outrate, sizeof(double)*num_nodes);
                bzero((char*)my_rxtr, sizeof(double)*num_nodes);
                // for sensing duty cycling
                my_disconnection = new bool[num_nodes];
                my_diedat = new double[num_nodes];
                my_areaid = new int[num_nodes];
                my_subareaid = new int[num_nodes];
                my_children_inarea = new int[num_nodes];
                my_childrensensing_inarea = new double[num_nodes];
                my_currentsensing = new double[num_nodes];
                my_totalareareq = new double[num_nodes];
                my_totalaraenode = new int[num_nodes];
                my_target = new double[num_nodes];
                my_subtreelowestlife = new double[num_nodes];
                my_subtreeEnergy = new double[num_nodes];
                my_subtreeCost = new double[num_nodes];
                my_subtreeWaste = new double[num_nodes];
                my_subtreeNode = new int[num_nodes];
                my_residual = new double[num_nodes];
                my_rescheduleTarget = new bool[num_nodes];
                my_totaltx = new int[num_nodes];
                my_totalrx = new int[num_nodes];
                my_totalsx = new double[num_nodes];
                my_totalareaalive = new int[num_nodes];
                bzero((char*)my_diedat, sizeof(double)*num_nodes);
                bzero((char*)my_disconnection, sizeof(bool)*num_nodes);
                bzero((char*)my_areaid, sizeof(int)*num_nodes);
                bzero((char*)my_subareaid, sizeof(int)*num_nodes);
                bzero((char*)my_children_inarea, sizeof(int)*num_nodes);
                bzero((char*)my_childrensensing_inarea, sizeof(double)*num_nodes);
                bzero((char*)my_currentsensing, sizeof(double)*num_nodes);
                bzero((char*)my_totalareareq, sizeof(double)*num_nodes);
                bzero((char*)my_totalaraenode, sizeof(int)*num_nodes);
                bzero((char*)my_target, sizeof(double)*num_nodes);
                bzero((char*)my_subtreelowestlife, sizeof(double)*num_nodes);
                bzero((char*)my_subtreeEnergy, sizeof(double)*num_nodes);
                bzero((char*)my_subtreeCost, sizeof(double)*num_nodes);
                bzero((char*)my_subtreeWaste, sizeof(double)*num_nodes);
                bzero((char*)my_residual, sizeof(double)*num_nodes);
                bzero((char*)my_subtreeNode, sizeof(int)*num_nodes);
                bzero((char*)my_rescheduleTarget, sizeof(bool)*num_nodes);
                bzero((char*)my_totaltx, sizeof(int)*num_nodes);
                bzero((char*)my_totalrx, sizeof(int)*num_nodes);
                bzero((char*)my_totalsx, sizeof(double)*num_nodes);
                bzero((char*)my_totalareaalive, sizeof(int)*num_nodes);
                my_etx = new double[num_nodes];
                my_edc = new double[num_nodes];
                my_eep = new double[num_nodes];
                bzero((char*)my_etx, sizeof(double)*num_nodes);
                bzero((char*)my_edc, sizeof(double)*num_nodes);
                bzero((char*)my_eep, sizeof(double)*num_nodes);
 for(int i=1;i<num_nodes;++i) my_etx[i]=my_edc[i]=my_eep[i]=99999999;
                // -Yang
                bzero((char*) min_hops, sizeof(int) * num_nodes * num_nodes);
                bzero((char*) mb_node, sizeof(MobileNode*) * num_nodes);
                bzero((char*) next_hop, sizeof(int) * num_nodes * num_nodes);

                instance_ = this;

                return TCL_OK;
            }

            if (strcasecmp(argv[1], "num_data_types") == 0) {
                assert(num_data_types == 0);
                num_data_types = atoi(argv[2]);
                assert(num_nodes > 0);
                assert(num_data_types > 0);
                source_table = new int*[num_data_types * num_nodes];
                sink_table = new int[num_data_types * num_nodes];
                num_send = new int[num_data_types];
                bzero((char*) source_table, sizeof(int *) * num_data_types * num_nodes);
                bzero((char*) sink_table, sizeof(int) * num_data_types * num_nodes);
                bzero((char*) num_send, sizeof(int) * num_data_types);
                return TCL_OK;
            }

            if (strcasecmp(argv[1], "new_node") == 0) {
              assert(num_nodes > 0);
              MobileNode *obj = (MobileNode *)TclObject::lookup(argv[2]);
              assert(obj != 0);
              assert(obj->address() < num_nodes);

              mb_node[obj->address()] = obj;
              return TCL_OK;
            }
    }
    else if (argc == 4) {
        if (strcasecmp(argv[1], "addUdpAgent") == 0) {
            int nodeid = atoi(argv[2]);
            sensor_tra_generator_[nodeid] = (UdpAgent*)TclObject::lookup(argv[3]);
            if( sensor_tra_generator_[nodeid] == 0 ){
                fprintf(stderr, "God::command failed to find the UdpAgent in command addUdpAgent with nodeid %d\n", nodeid);
                return TCL_ERROR;
            } else {
                return TCL_OK;
            }
        }
        if (strcasecmp(argv[1], "is_reachable") == 0) {
            int n1 = atoi(argv[2]);
            int n2 = atoi(argv[3]);

            if (IsReachable(n1,n2) == true) {
                tcl.result("1");
            } else {
                tcl.result("0");
            }
            return TCL_OK;
        }
        // We can add source from tcl script or call AddSource directly.
        if (strcasecmp(argv[1], "add_source") == 0) {
            int dt = atoi(argv[2]);
            int srcid = atoi(argv[3]);

            AddSource(dt, srcid);
            return TCL_OK;
        }
        // We can add sink from tcl script or call AddSink directly.
        if (strcasecmp(argv[1], "add_sink") == 0) {
            int dt = atoi(argv[2]);
            int skid = atoi(argv[3]);

            AddSink(dt, skid);
            return TCL_OK;
        }
        if (strcasecmp(argv[1], "setareaid") == 0) {
            int nodeid = atoi(argv[2]);
            int areaid = atoi(argv[3]);
            my_areaid[nodeid] = areaid;

            return TCL_OK;
        }
    }
    else if(argc == 5) {
        /* load for grid-based adaptive fidelity */
        if (strcmp(argv[1], "load_grid") == 0) {
            if(load_grid(atoi(argv[2]), atoi(argv[3]), atoi(argv[4])))
                return TCL_ERROR;
            return TCL_OK;
        }

        if (strcasecmp(argv[1], "set-dist") == 0) {
            int i = atoi(argv[2]);
            int j = atoi(argv[3]);
            int d = atoi(argv[4]);

            assert(i >= 0 && i < num_nodes);
            assert(j >= 0 && j < num_nodes);

            if (active == true) {
                if (NOW > prev_time) {
                    ComputeRoute();
                }
            }
            else {
              min_hops[i*num_nodes+j] = d;
              min_hops[j*num_nodes+i] = d;
            }

            // The scenario file should set the node positions
            // before calling set-dist !!

            assert(min_hops[i * num_nodes + j] == d);
                        assert(min_hops[j * num_nodes + i] == d);
                        return TCL_OK;
        }
    }
    else if(argc == 6) {
        if (strcasecmp(argv[1], "triggerEvent") == 0) {
            if( random_event_flag_ == false ){
              // traffic has been stopped, ignore the event
              return TCL_OK;
            }
            double x = atof(argv[2]);
            double y = atof(argv[3]);
            double radius = atof(argv[4]);
            int pktsize = atoi(argv[5]);
            generateTrafficFromEvent(x,y,radius,pktsize);
            return TCL_OK;
        }
    }
    return BiConnector::command(argc, argv);
}

void God::generateTrafficFromEvent(double x, double y, double radius, int pktsize){
    double radiussquare = radius * radius;
    // skip node 0 (the dumb node)
    for(int i = 1; i < num_nodes; i++) {
        if( sensor_tra_generator_[i] ){
            if( (x-mb_node[i]->X())*(x-mb_node[i]->X()) + (y-mb_node[i]->Y())*(y-mb_node[i]->Y()) < radiussquare ){
                // in range, sendpkt
                sensor_tra_generator_[i]->sendmsg( pktsize, NULL, NULL );
            }
        }
    }
}
// Yang- 04/25/2010
void God::addDutyOnTime(int nodeid, double time) {
    if (nodeid<0 || nodeid> num_nodes || dutyontime_==0) return;
    dutyontime_[nodeid] += time;
}
void God::logDutyOntime(){
    if (dutyontime_==0 || ctrace_==0) return;
    for (int i=0;i<num_nodes;i++) {
        double ratio = dutyontime_[i]*100.0/NOW;
        double initial = initialEng[i];
        double current = lastUpdateEng[i];
        double lifetime = current;
        //ctrace_->log("node %d initial %f lifetime %f dutyontime %f ratio %f\% \n", i, initial,lifetime,dutyontime_[i],ratio);
        //ctrace_->log("node %d areaid %d subaraeid %d parentid %d died at %f\n", i, my_areaid[i], my_subareaid[i], current_parent[i],my_disconnection[i]?my_diedat[i]:NOW);
        //ctrace_->log("node %d totalchildrensensing %f currentsensing %f\n",i, my_childrensensing_inarea[i], my_currentsensing[i]);
    }
/*
    for (int i=1;i<num_nodes;i++) {
        //ctrace_->log("node %d eep %f edc %f etx %f\n", i, my_eep[i], my_edc[i], my_etx[i]);
        printf("node %d eep %f edc %f etx %f forwarderset ", i, my_eep[i], my_edc[i], my_etx[i]);
        for (int j=0;j<num_nodes;j++) {
            if (j!=i && getAllowAnycast(i,j)) {
                printf("%d ", j);
            }
        }
        printf("\n");
    }
*/
}

// for jros
void God::notifyDisconnection(int nodeid) {
    int parent = current_parent[nodeid];
    while(parent > 0 && !my_disconnection[parent] && my_areaid[nodeid] == my_areaid[parent]) {
        my_children_inarea[parent] -= my_children_inarea[nodeid];
        my_subtreeNode[parent] -= my_subtreeNode[nodeid];
        my_childrensensing_inarea[parent] -= my_childrensensing_inarea[nodeid];
        parent = current_parent[parent];
    }
}

void God::notifyChangeParent(int nodeid, int oldparent, int newparent){
    int parent = oldparent;
    while(parent > 0 && !my_disconnection[parent] && my_areaid[nodeid] == my_areaid[parent]) {
        my_children_inarea[parent] -= my_children_inarea[nodeid];
        my_subtreeNode[parent] -= my_subtreeNode[nodeid];
        my_childrensensing_inarea[parent] -= my_childrensensing_inarea[nodeid];
        parent = current_parent[parent];
    }
    parent = newparent;
    while(parent > 0 && !my_disconnection[parent] && my_areaid[nodeid] == my_areaid[parent]) {
        my_children_inarea[parent] += my_children_inarea[nodeid];
        my_subtreeNode[parent] += my_subtreeNode[nodeid];
        my_childrensensing_inarea[parent] += my_childrensensing_inarea[nodeid];
        parent = current_parent[parent];
    }
}

void God::reduceSensingCost(int nodeid, double cost){
    mb_node[nodeid]->energy_model()->DecrTxEnergy(1,cost);
    my_energy[nodeid] = mb_node[nodeid]->energy_model()->energy();
    if (my_energy[nodeid] <= 0) {
        if (!my_disconnection[nodeid]) {
            my_diedat[nodeid]=NOW;
            notifyDisconnection(nodeid);
            printf("node %d disconnection tx %d rx %d sx %f, out of energy after reducingcost @ %f\n",
                nodeid, my_totaltx[nodeid], my_totalrx[nodeid], my_totalsx[nodeid],NOW);
        }
        my_disconnection[nodeid] = true;
    }
}

// for desa
/*
double God::getPrr(int nodeid, int parent){
    return neighborPrr[nodeid][parent];
}

void God::setPrr(int nodeid, int parent, double prr){
    neighborPrr[nodeid][parent] = prr;
}

bool God::getAllowAnycast(int src, int dst){
    return allowAnycast[src][dst];
}

void God::setAllowAnycast(int src, int dst, bool value){
    allowAnycast[src][dst] = value;
}
double God::getETX(int nodeid, double value){
    if (value<0) return my_etx[nodeid];
    else my_etx[nodeid] = value;
}
double God::getEEP(int nodeid, double value){
    if (value<0) return my_eep[nodeid];
    else my_eep[nodeid] = value;
}
double God::getEDC(int nodeid, double value){
    if (value<0) return my_edc[nodeid];
    else my_edc[nodeid] = value;
}
*/
