/*
 * nrrapplication.cc
 *
 *  Created on: Aug 19, 2009
 *      Author: yang
 */

#include "rilapp.h"
#include "god.h"
#include <vector>
#include <map>

static class RILAppClass : public TclClass {
public:
    RILAppClass() : TclClass("Application/RILApp") {}
    TclObject* create(int argc, const char*const* argv) {
        assert(argc==5);
        return (new RILApp(atoi(argv[4])));
    }
}class_RILApp;

RILApp::RILApp(int nodeid):dataT_(0), myID(nodeid), packetseq(0){
    bind("DataRate", &dataInterval_);
    bind("SensingReq", &sensingReq_);
    bind("ScheduleMethod", &scheduleMethod_);
    bind("FixedPercent", &fixedPercent_);
    txcost = 0.002;   // tx cost per second given 100% duty cycle sensing activities
    rxcost = 0.001;   // rx cost per second given 100% duty cycle sensing activities
    sensingcost = 0.002;  // sensing cost per second given 100% duty cycle sensing activities
    fixcost = sensingcost*fixedPercent_; // fixed cost per second
    periodsensing = 0;
}

RILApp::~RILApp(){
    if (dataT_) dataT_->force_cancel();
    delete dataT_;
    if (controlT_) controlT_->force_cancel();
    delete controlT_;
}



int RILApp::command(int argc, const char*const* argv) {
    if (argc == 2) {
        if (strcmp(argv[1], "start") == 0) {
            start();
            return (TCL_OK);
        }
        else if (strcmp(argv[1], "stop") == 0) {
            stop();
            return (TCL_OK);
        }
    }
    return (Application::command(argc, argv));
}

double RILApp::next_interval() {
    double dt = God::instance()->my_currentsensing[myID];
    if (dt>0.00001) dt = dataInterval_/God::instance()->my_currentsensing[myID];
    else dt = 0;
    return dt;
    //return Random::uniform(dt*0.9,dt*1.1);
    //return NOW+Random::uniform(dt*0.9,dt*1.1);
}


void RILApp::JROSAllocate_single() {
    int id = 0; double temp = 0;
    for (std::vector<int>::iterator it = onehopchildren.begin(); it != onehopchildren.end(); ++it) {
        id = *it;
        if (God::instance()->my_rescheduleTarget[id]) continue;
        if (God::instance()->my_lifetime[myID] < God::instance()->my_lifetime[id]) {
            temp = God::instance()->my_currentsensing[myID];
            if (God::instance()->my_currentsensing[myID] < temp) {
                temp = God::instance()->my_currentsensing[myID];
            }
            if (God::instance()->my_currentsensing[id] + temp > 1) {
                temp = 1 - God::instance()->my_currentsensing[id];
            }
            if (temp > 0.01) temp = 0.01;
            if (temp <= 0.0001) continue;
        }
        else {
            bool pushtoleaf = true;
            int subareaid = God::instance()->my_subareaid[myID];
            temp = 0;
            if (
                God::instance()->my_totalareaalive[God::instance()->my_areaid[id]] - God::instance()->my_children_inarea[id] < ceil(God::instance()->my_totalareareq[subareaid])+1
                //|| God::instance()->my_children_inarea[subareaid] - God::instance()->my_children_inarea[id] < ceil(God::instance()->my_target[subareaid])
                //|| God::instance()->my_subtreeWaste[id] > 0
                //|| fabs(God::instance()->my_lifetime[id] - God::instance()->my_subtreelowestlife[id]) <= 0.0001
                )
            {
                if (God::instance()->my_lifetime[myID] > God::instance()->my_lifetime[id]) {
                    temp = God::instance()->my_currentsensing[id];
                    if (God::instance()->my_currentsensing[myID] + temp > 1) {
                        temp = 1 - God::instance()->my_currentsensing[myID];
                    }
                    if (God::instance()->my_currentsensing[id] < temp) {
                        temp = God::instance()->my_currentsensing[id];
                    }
                    if (temp <= 0.0001) continue;
                    if (temp > 0.01) temp = 0.01;
                    temp = -1.0*temp;
                }
                pushtoleaf = false;
            }
            if (God::instance()->my_currentsensing[id] < 1 && pushtoleaf) {
                temp = 1 - God::instance()->my_currentsensing[id];
                if (God::instance()->my_currentsensing[myID] < temp) {
                    temp = God::instance()->my_currentsensing[myID];
                }
                if (temp > 0.01) temp = 0.01;
                if (temp <= 0.0001) continue;
            }
            else if (temp>=0 && fabs(God::instance()->my_currentsensing[id]-1)<=0.0001) continue;
        }

        God::instance()->my_currentsensing[myID] -= temp;
        God::instance()->my_target[id] += temp;
        God::instance()->my_currentsensing[id] += temp;
        God::instance()->my_childrensensing_inarea[id] += temp;
        printf("node %d move %f from node %d cur %f life %f to node %d life %f cur %f subsensing %f hop %d @ %f\n",
                myID, temp, myID, God::instance()->my_currentsensing[myID], God::instance()->my_lifetime[myID],
                id, God::instance()->my_lifetime[id],  God::instance()->my_currentsensing[id],
                God::instance()->my_childrensensing_inarea[id], God::instance()->current_hop[myID],
                NOW);
    }
}

void RILApp::JROSAllocate_multiple() {
    if (onehopchildren.size() < 2) return;

    std::multimap<double, int> positiveset;
    std::multimap<double, int> negativeset;
    std::multimap<double, int> criticalset;
    std::vector<int> sortedset;
    int totalnode = 0;
    double target = 0;
    int id = 0; int subareaid = -1;
    for (std::vector<int>::iterator it = onehopchildren.begin(); it != onehopchildren.end(); ++it) {
        id = *it;
        totalnode += God::instance()->my_children_inarea[id];
        target += God::instance()->my_target[id];
    }

    for (std::vector<int>::iterator it = onehopchildren.begin(); it != onehopchildren.end(); ++it) {
        id = *it;
        subareaid = God::instance()->my_subareaid[id];
        totalnode -= God::instance()->my_children_inarea[id];
        if (
            God::instance()->my_totalareaalive[God::instance()->my_areaid[id]] - God::instance()->my_children_inarea[id] < ceil(God::instance()->my_totalareareq[id])
            //|| God::instance()->my_children_inarea[subareaid] - God::instance()->my_children_inarea[id] < ceil(God::instance()->my_target[subareaid])
            || totalnode < ceil(target)
        ) {
            criticalset.insert(std::pair<double,int>(God::instance()->my_subtreelowestlife[id], id));
            printf("node %d add node %d to critical subtree %d inarea %d residual %f alive %d total %d target %f @ %f\n", myID,id, God::instance()->my_subtreeNode[id],God::instance()->my_children_inarea[id],God::instance()->my_residual[id],God::instance()->my_totalareaalive[God::instance()->my_areaid[id]],totalnode, target,NOW);
        }
        else {
            if (God::instance()->my_residual[id] > 0.0001) {
                positiveset.insert(std::pair<double,int>(God::instance()->my_residual[id], id));
                printf("node %d add node %d to positive subtree %d inarea %d residual %f @ %f\n", myID,id, God::instance()->my_subtreeNode[id],God::instance()->my_children_inarea[id],God::instance()->my_residual[id],NOW);
            }
            else {
                negativeset.insert(std::pair<double,int>(God::instance()->my_subtreeWaste[id], id));
                printf("node %d add node %d to negative subtree %d inarea %d waste %f @ %f\n", myID,id, God::instance()->my_subtreeNode[id],God::instance()->my_children_inarea[id],God::instance()->my_subtreeWaste[id],NOW);
            }
        }
    }

    for (std::multimap<double, int>::reverse_iterator it = positiveset.rbegin(); it != positiveset.rend(); ++it) {
        sortedset.push_back(it->second);
    }
    for (std::multimap<double, int>::iterator it = negativeset.begin(); it != negativeset.end(); ++it) {
        sortedset.push_back(it->second);
    }
    for (std::multimap<double, int>::reverse_iterator it = criticalset.rbegin(); it != criticalset.rend(); ++it) {
        sortedset.push_back(it->second);
    }
    int i = 0, j = sortedset.size()-1, dst = 0, src = 0;
    double temp = 0;
    while(i<j) {
        dst = sortedset[i];
        src = sortedset[j];
        temp = God::instance()->my_children_inarea[dst] - God::instance()->my_childrensensing_inarea[dst];
        if (temp<=0.0001 || God::instance()->my_rescheduleTarget[dst]) {
            ++i;
            continue;
        }
        if (temp >= 0.01) temp = 0.01;

        if (God::instance()->my_childrensensing_inarea[src] < temp) {
            temp = God::instance()->my_childrensensing_inarea[src];
        }
        if (God::instance()->my_target[dst] + temp > God::instance()->my_children_inarea[dst]) {
            temp = God::instance()->my_children_inarea[dst] - God::instance()->my_target[dst];
        }

        if (temp <= 0.0001 || God::instance()->my_rescheduleTarget[src]) {
            --j;
            continue;
        }
        // now move sensing activities from src node to dst node
        God::instance()->my_rescheduleTarget[src] = true;
        God::instance()->my_rescheduleTarget[dst] = true;
        God::instance()->my_target[src] -= temp;
        God::instance()->my_target[dst] += temp;
        // if I can change it now, change it directly
        if (God::instance()->my_currentsensing[src]>=temp && God::instance()->my_currentsensing[dst]+temp<=1) {
            God::instance()->my_currentsensing[src]-=temp;
            God::instance()->my_currentsensing[dst]+=temp;
            God::instance()->my_childrensensing_inarea[src]-=temp;
            God::instance()->my_childrensensing_inarea[dst]+=temp;
            God::instance()->my_rescheduleTarget[src] = false;
            God::instance()->my_rescheduleTarget[dst] = false;
        }
        printf("node %d move sensing %f from node %d life %f waste %f sen %f %d %f to node %d life %f waste %f sen %f %d %f @ %f\n",
                myID, temp, src, God::instance()->my_lifetime[src], God::instance()->my_subtreeWaste[src], God::instance()->my_target[src],
                God::instance()->my_children_inarea[src], God::instance()->my_childrensensing_inarea[src],
                dst, God::instance()->my_lifetime[dst], God::instance()->my_subtreeWaste[dst], God::instance()->my_target[dst],
                God::instance()->my_children_inarea[dst], God::instance()->my_childrensensing_inarea[dst], NOW);
        --j;
        //if (God::instance()->my_subtreeWaste[dst] > 0) return;
    }

}

void RILApp::BalanceAllocate() {
    int id = 0; double temp = 0;
    for (std::vector<int>::iterator it = onehopchildren.begin(); it != onehopchildren.end(); ++it) {
        id = *it;
        temp = 0.005;
        if (God::instance()->my_rescheduleTarget[id]) continue;
        if (God::instance()->my_lifetime[myID] > God::instance()->my_lifetime[id]) {
            if (God::instance()->my_currentsensing[myID] + temp > 1) temp = 1- God::instance()->my_currentsensing[myID];
            if (temp < 0) temp = 0;
            if (God::instance()->my_currentsensing[id] < temp) temp = God::instance()->my_currentsensing[id];
            God::instance()->my_currentsensing[myID] += temp;
            God::instance()->my_currentsensing[id] -= temp;
            God::instance()->my_target[id] -= temp;
            God::instance()->my_childrensensing_inarea[id] -= temp;
        }
        else if (God::instance()->my_lifetime[myID]< God::instance()->my_lifetime[id]) {
            if (God::instance()->my_currentsensing[id] + temp > 1) temp = 1 - God::instance()->my_currentsensing[id];
            if (temp < 0) temp = 0;
            if (God::instance()->my_currentsensing[myID] < temp) temp = God::instance()->my_currentsensing[myID];
            God::instance()->my_currentsensing[myID] -= temp;
            God::instance()->my_currentsensing[id] += temp;
            God::instance()->my_target[id] += temp;
            God::instance()->my_childrensensing_inarea[id] += temp;
            temp = temp*-1;
        }
        else temp = 0;
        if (fabs(temp) <= 0.00001) continue;
        // balance allocate wont affect the value of my_totalchildrensensing
        printf("node %d move sensing %f from node %d life %f cur %f to node %d life %f cur %f@ %f\n",
                myID, temp, myID, God::instance()->my_lifetime[myID], God::instance()->my_currentsensing[myID], id, God::instance()->my_lifetime[id], God::instance()->my_currentsensing[myID], NOW);
    }
}

void RILApp::FixAllocate(){
    // assuming that my_target has been determined and needs to be updatd only when notified
    if (God::instance()->my_rescheduleTarget[myID] != true) return;
    for(vector<int>::iterator it=onehopchildren.begin(); it!=onehopchildren.end();++it) {
        if (God::instance()->my_rescheduleTarget[*it]) {
            printf("node %d child %d is pending scheduled @ %f\n",myID,*it,NOW);
            return;
        }
    }
    God::instance()->my_rescheduleTarget[myID] = false;

    if (God::instance()->my_target[myID] > God::instance()->my_children_inarea[myID]) {
        // my child might have changed parent, and the target value might not be correct, do my best
        printf("node %d target changed from %f to %d @ %f\n",myID, God::instance()->my_target[myID],God::instance()->my_children_inarea[myID],NOW);
        God::instance()->my_target[myID] = God::instance()->my_children_inarea[myID];
    }
    double left = God::instance()->my_target[myID] - God::instance()->my_childrensensing_inarea[myID];
    if (fabs(left)<=0.0001) return;
    double areasen = God::instance()->my_childrensensing_inarea[myID] - God::instance()->my_currentsensing[myID];
    // allocate the minimum to self, and the rest to child ? might not be good
    God::instance()->my_currentsensing[myID] = God::instance()->my_target[myID] - (God::instance()->my_children_inarea[myID]-1);
    if (God::instance()->my_currentsensing[myID] < 0) God::instance()->my_currentsensing[myID] = 0;
    if (God::instance()->my_currentsensing[myID] > 1) God::instance()->my_currentsensing[myID] = 1;
    // assume that childsensing in area has been changed, though they may happen later
    God::instance()->my_childrensensing_inarea[myID] = God::instance()->my_target[myID];
    left = God::instance()->my_target[myID] - God::instance()->my_currentsensing[myID];

    // now we have the target value for the whole subtree
    if (onehopchildren.empty()) return;
    printf("node %d fixeallocate target %f cur %f inarea %f left %f @ %f\n",myID, God::instance()->my_target[myID], God::instance()->my_currentsensing[myID],God::instance()->my_childrensensing_inarea[myID],left,NOW);

    double areareq = left;
    double curTarget = 0, temp = 0;
    int id = 0;
    while (true) {
        for(vector<int>::iterator it=onehopchildren.begin(); it!=onehopchildren.end();++it) {
            id = *it;
            if (!God::instance()->my_rescheduleTarget[id]) God::instance()->my_target[id] = 0;
            God::instance()->my_rescheduleTarget[id] = true;
            if (fabs(God::instance()->my_target[id] - God::instance()->my_children_inarea[id]) <= 0.0001) {
                continue;
            }

            if (areasen > 0) {
                temp = left*God::instance()->my_childrensensing_inarea[id]/areasen;
            }
            else {
                temp = left*God::instance()->my_children_inarea[id]/(God::instance()->my_children_inarea[myID]-1);
            }
            if (temp < 0.005) temp = 0.005;
            if (God::instance()->my_target[id] + temp > God::instance()->my_children_inarea[id]) {
                temp = God::instance()->my_children_inarea[id] - God::instance()->my_target[id];
            }
            if (curTarget + temp > areareq) {
                temp = areareq - curTarget;
            }
            God::instance()->my_target[id] += temp;
            curTarget += temp;
            printf("node %d reschedule node %d target %f children %d temp %f @ %f\n",myID, id, God::instance()->my_target[id], God::instance()->my_children_inarea[id], temp,NOW);
        }
        left = areareq - curTarget;
        if (left <= 0.0001) break;
    }
}

void RILApp::UpdateChildrenInfo() {
    int curparent = God::instance()->current_parent[myID];
    if (God::instance()->my_disconnection[curparent]) {
        // if my parent is disconnected, send a data to find a new parent
        RILAppMessageT_ msg;
        produceMsg(&msg, DATA_SENSING);
        sendMsg(&msg);
        return;
    }

    // update my current information
    God::instance()->my_subtreelowestlife[myID] = God::instance()->my_lifetime[myID];
    God::instance()->my_subtreeWaste[myID] = 0;
    God::instance()->my_residual[myID] = 0;
    if (God::instance()->current_parent[myID] == 0) {
        God::instance()->my_subareaid[myID] = myID;
        God::instance()->current_hop[myID] = 1;
        God::instance()->min_upstream_id[myID] = myID;
    }
    else {
        if (God::instance()->my_lifetime[myID] < God::instance()->my_lifetime[God::instance()->min_upstream_id[curparent]]) {
            God::instance()->min_upstream_id[myID] = myID;
        }
        else {
            God::instance()->min_upstream_id[myID] = God::instance()->min_upstream_id[curparent];
        }
    }

    int subtreenode = 0;
    double subtreeenergy = 0;
    double subtreecost = 0;

    double subtreesensing_inarea = 0;
    double subtreenode_inarea = 0;
    onehopchildren.clear();
    std::multimap<double, double>lifewasteset;
    for (int i=1; i< God::instance()->my_totalnodes; ++i) {
        if (i != myID
            && God::instance()->current_parent[i] == myID
            && !God::instance()->my_disconnection[i]
        ){
            // current alive nodes of my direct children
            subtreenode += God::instance()->my_subtreeNode[i];
            subtreeenergy += God::instance()->my_subtreeEnergy[i];
            subtreecost += God::instance()->my_energy[i]/God::instance()->my_lifetime[i];
#if 0
            if (God::instance()->my_subtreelowestlife[i] >= God::instance()->my_lifetime[myID]) {
                
                if (God::instance()->my_residual[myID]<=0.0001)
                    God::instance()->my_subtreeWaste[myID] += God::instance()->my_subtreeEnergy[i] - God::instance()->my_lifetime[myID]*God::instance()->my_subtreeCost[i];
                //God::instance()->my_residual[myID] = 0;
                iamlower = true;
            }
            else {
                //if (!iamlower) {
                    double re = (God::instance()->my_lifetime[myID] - God::instance()->my_subtreelowestlife[i])*God::instance()->my_energy[myID]/God::instance()->my_lifetime[myID];
                    if (God::instance()->my_residual[myID]<=0.0001 || God::instance()->my_residual[myID] > re)God::instance()->my_residual[myID] = re;
                    God::instance()->my_subtreeWaste[myID] = 0;
                //}
                //God::instance()->my_subtreeWaste[myID] += God::instance()->my_subtreeWaste[i];
                if (God::instance()->my_subtreelowestlife[i] < God::instance()->my_subtreelowestlife[myID]) {
                    God::instance()->my_subtreelowestlife[myID] = God::instance()->my_subtreelowestlife[i];
                }
            }
#endif
            lifewasteset.insert(std::pair<double,double>(God::instance()->my_subtreelowestlife[i], God::instance()->my_subtreeWaste[i]));
            if (God::instance()->my_areaid[i] != God::instance()->my_areaid[myID]) {
                God::instance()->my_subareaid[i] = i;
            }
            else {
                God::instance()->my_subareaid[i] = God::instance()->my_subareaid[myID];
            }
            God::instance()->current_hop[i] = God::instance()->current_hop[myID] + 1;

            if (God::instance()->my_areaid[i] == God::instance()->my_areaid[myID]) {
                // use child's target as the actual sensing if this not is been scheduled, else use the actual sensing
                if (God::instance()->my_rescheduleTarget[i]) subtreesensing_inarea += God::instance()->my_target[i];
                else subtreesensing_inarea += God::instance()->my_childrensensing_inarea[i];
                subtreenode_inarea += God::instance()->my_children_inarea[i];
                // only put node in my area to my onehopchildren set for scheduling
                onehopchildren.push_back(i);
            }
        }
    }
    if (!lifewasteset.empty()) {
        if (God::instance()->my_lifetime[myID] > lifewasteset.begin()->first) {
            God::instance()->my_subtreelowestlife[myID] = lifewasteset.begin()->first;
            God::instance()->my_subtreeWaste[myID] = lifewasteset.begin()->second;
            God::instance()->my_residual[myID] = subtreeenergy - God::instance()->my_subtreeWaste[myID] - God::instance()->my_subtreelowestlife[myID]*subtreecost;
        }
        else {
            God::instance()->my_subtreelowestlife[myID] = God::instance()->my_lifetime[myID];
            God::instance()->my_subtreeWaste[myID] = subtreeenergy - God::instance()->my_lifetime[myID]*subtreecost;
            God::instance()->my_residual[myID] = 0;
        }
        if (God::instance()->my_residual[myID] < 0) God::instance()->my_residual[myID] = 0;
        if (God::instance()->my_subtreeWaste[myID] < 0) God::instance()->my_subtreeWaste[myID] = 0;
    }
    // add my information
    God::instance()->my_children_inarea[myID] = subtreenode_inarea + 1;
    God::instance()->my_childrensensing_inarea[myID] = subtreesensing_inarea + God::instance()->my_currentsensing[myID];
    God::instance()->my_subtreeNode[myID] = subtreenode + 1;
    God::instance()->my_subtreeEnergy[myID] = subtreeenergy + God::instance()->my_energy[myID];
    God::instance()->my_subtreeCost[myID] = subtreecost + God::instance()->my_energy[myID]/God::instance()->my_lifetime[myID];
    printf("node %d parent %d cur %f subtree %f child %d life %f energy %f residual %f waste %f lowest %f @ %f\n",
        myID, God::instance()->current_parent[myID], God::instance()->my_currentsensing[myID], God::instance()->my_childrensensing_inarea[myID], God::instance()->my_children_inarea[myID],
        God::instance()->my_lifetime[myID], God::instance()->my_energy[myID], God::instance()->my_residual[myID],God::instance()->my_subtreeWaste[myID], God::instance()->my_subtreelowestlife[myID],NOW);
}

void RILApp::VerifyInitialization() {
    if (God::instance()->netinitialized) return;
    int totalnodes = 1;
    for (int i=1; i < God::instance()->my_totalnodes; i++) {
        if (God::instance()->current_parent[i] == -1) return;
        if (God::instance()->my_areaid[i] != God::instance()->my_areaid[God::instance()->current_parent[i]]) {
            totalnodes += God::instance()->my_children_inarea[i];
        }
        God::instance()->my_target[i] = God::instance()->my_childrensensing_inarea[i];
    }
    God::instance()->netinitialized = (totalnodes == God::instance()->my_totalnodes);
    if (God::instance()->netinitialized) {
        printf("network is initialized @ %f\n",NOW);
    }
}

void RILApp::timeout(int type) {
    if (type == CTIMER) {
        if (God::instance()->isSource(myID)) {
            UpdateAndSchedule();
            God::instance()->reduceSensingCost(myID, sensingcost*God::instance()->my_currentsensing[myID]);
            God::instance()->reduceSensingCost(myID, fixcost);
            God::instance()->my_totalsx[myID]+=sensingcost*God::instance()->my_currentsensing[myID];
            periodsensing += God::instance()->my_currentsensing[myID];
            if (periodsensing >= dataInterval_) {
                periodsensing -= dataInterval_;
                RILAppMessageT_ msg;
                produceMsg(&msg, DATA_SENSING);
                sendMsg(&msg);
                God::instance()->reduceSensingCost(myID, txcost);
            }
            controlT_->resched(1);
        }
        else {
            if (!God::instance()->netinitialized) {
                VerifyInitialization();
                controlT_->resched(1);
            }
            else {
                SinkMonitor(sensingReq_);
                controlT_->resched(5);
            }
        }
    }
}

void RILApp::UpdateAndSchedule() {
    // disconnected node don't do schedule
    if (God::instance()->my_disconnection[myID]) {
        God::instance()->my_children_inarea[myID] = 0;
        God::instance()->my_childrensensing_inarea[myID] = 0;
        God::instance()->my_subtreeNode[myID] = 0;
        return;
    }
    // update children info at first
    UpdateChildrenInfo();
    // reschedule after network is initialized
    if (God::instance()->netinitialized) {
        FixAllocate();
        if (scheduleMethod_ == 1) {
            JROSAllocate_single();
            JROSAllocate_multiple();
        }
        else if (scheduleMethod_ == 2) {
            BalanceAllocate();
        }
        else {
            exit(-1);
        }
    }
}

// process received nrrapp packet
void RILApp::process_data(int size, AppData* data) {
    PacketData* p = (PacketData*)data;
    RILAppMessageT_* cappmsgp = (RILAppMessageT_*)p->data();
    recvMsg(cappmsgp);
    // update and schedule after receiving a packet
    UpdateAndSchedule();
    return;
}

void RILApp::start() {
    God::instance()->ctrace()->log("node %d start app scheduling %d @ %f\n", myID, scheduleMethod_, NOW);
    controlT_ = new RILAppTimer(this, CTIMER);
    if (God::instance()->isSource(myID)) {
        dataT_ = new RILAppTimer(this, DTIMER);
        God::instance()->my_currentsensing[myID] = sensingReq_;
        God::instance()->my_target[myID] = 0;
        God::instance()->my_subtreelowestlife[myID] = God::instance()->my_lifetime[myID];
        God::instance()->current_parent[myID] = -1;
        God::instance()->my_subtreeEnergy[myID] = God::instance()->my_energy[myID];
        God::instance()->my_subtreeCost[myID] = 0;
        God::instance()->my_subtreeWaste[myID] = 0;
        God::instance()->min_upstream_id[myID] = myID;
        dataT_->sched(dataInterval_);
    }
    else {
        // sink needs to verify network initialization status
        God::instance()->my_areaid[myID] = -1;
        God::instance()->my_totalareareq[myID] = sensingReq_*(God::instance()->my_totalnodes-1);
        God::instance()->my_totalaraenode[myID] = God::instance()->my_totalnodes-1;
    }
    God::instance()->current_hop[myID] = 0;
    for (int i=1;i<God::instance()->my_totalnodes;i++) {
        if (God::instance()->my_areaid[myID] == God::instance()->my_areaid[i]) {
            God::instance()->my_totalareareq[myID] += sensingReq_;
            God::instance()->my_totalaraenode[myID] += 1;
            //God::instance()->my_totalareaalive[God::instance()->my_areaid[myID]] += 1;
        }
    }

    controlT_->sched(1);

    printf("node %d areaid %d areanode %d areasenreq %f alive %d energy %f totalarea %d \n",
        myID, God::instance()->my_areaid[myID], God::instance()->my_totalaraenode[myID],
        God::instance()->my_totalareareq[myID], God::instance()->my_totalareaalive[God::instance()->my_areaid[myID]],God::instance()->my_energy[myID], totalarea);
}

void RILApp::stop() {
    God::instance()->ctrace()->log("node %d stop app @ %f\n", myID, NOW);
    if (dataT_ != 0) dataT_->stoptimer();
    if (controlT_ !=0) controlT_->stoptimer();
}

void RILApp::produceMsg(RILAppMessageT_* msg, int msgtype) {
	msg->source = myID;
	msg->msgType = msgtype;
	msg->timestamp = NOW;
	msg->seq = packetseq++;
	return;
}

void RILApp::sendMsg(RILAppMessageT_* msg) {
	msg->dest = 0;
    msg->netType = UCAST;
    PacketData* data = new PacketData(sizeof(RILAppMessageT_));
    memcpy(data->data(), msg, sizeof(RILAppMessageT_));
    agent_->sendmsg(sizeof(RILAppMessageT_), data);
    God::instance()->my_totaltx[myID]++;
}

void RILApp::recvMsg(RILAppMessageT_* msg) {
    if (msg == 0) return;
    if (God::instance()->isSource(myID)) {
        God::instance()->reduceSensingCost(myID, (txcost+rxcost));
        God::instance()->my_totalrx[myID]++;
        God::instance()->my_totaltx[myID]++;
    }
    #if 0
    if (msg->msgType == DATA_SENSING) {
        int src = msg->source;
        int delay = (int)((NOW-msg->timestamp)*1000.0);
        int hop = God::instance()->current_hop[src];
        God::instance()->ctrace()->log("from %d seq %d delay %d hop %d @ %f\n", src, msg->seq,delay,hop,NOW);
    }
    #endif
}

void RILApp::dumpMsg(RILAppMessageT_* msg, bool yes) {

}

void RILAppTimer::expire(Event *) {
	if (continueRunning) {
		appinstance_->timeout(this->type);
	}
}

void RILApp::SinkMonitor(double nodereq) {

    int areaalive[totalarea] = {0};
    int areatotal[totalarea] = {0};
    int directarea[totalarea] = {0};
    double areasensing[totalarea] = {0.0};
    map< int, vector<int> > rescheduleArea;
    map< int, vector<int> > rescheduleDirectArea;

    int areaid = -1;
    for (int i=1; i<God::instance()->my_totalnodes; i++) {
        areaid = God::instance()->my_areaid[i];
        if (!God::instance()->my_disconnection[i]) {
            if (areaid != God::instance()->my_areaid[God::instance()->current_parent[i]]) {
                rescheduleArea[areaid].push_back(i);
                areaalive[areaid] += God::instance()->my_children_inarea[i];
                areasensing[areaid] += God::instance()->my_childrensensing_inarea[i];
                if (God::instance()->current_parent[i] == 0) {
                    directarea[areaid] += God::instance()->my_children_inarea[i];
                }
            }
        }
        areatotal[areaid]++;
    }
    double areareq = 0;
    for (int i=0; i<totalarea; i++) {
        areareq = areatotal[i]*nodereq;
        if (areareq <= 0) continue;
        if (areaalive[i] < ceil(areareq)) {
            printf("detect sensing requirement violation at area %d req %f actual %f alive %d @ %f\n",
                    i, areareq, areasensing[i], areaalive[i], NOW);
            God::instance()->logDutyOntime();
            exit(-1);
        }
        // the current sensng in the area is the same as requirement, no need to reschedule
        if (fabs(areasensing[i] - areareq) <= 0.001) {
            // but if this area is a direct area of sink, reschedule it
            if (directarea[i] == areaalive[i]) {
                rescheduleDirectArea[i] = rescheduleArea[i];
            }
            rescheduleArea.erase(i);
        }
        God::instance()->my_totalareaalive[i] =  areaalive[i];
        printf("area %d alive %d req %f sensing %f @ %f\n",i, God::instance()->my_totalareaalive[i], areareq, areasensing[i], NOW);
    }

    double curTarget= 0, left = 0, temp = 0; int id = 0;
    for(map< int, vector<int> >::iterator it=rescheduleArea.begin(); it!=rescheduleArea.end(); ++it) {
        areaid = it->first;
        left = areareq = areatotal[areaid]*nodereq;
        curTarget = temp = 0;
        printf("sink reschedule area %d sensing %f left %f @ %f\n", areaid, areasensing[areaid], left, NOW);
        #if 1
        while (left > 0.0001) {
            for(vector<int>::iterator it2=(it->second).begin(); it2!=(it->second).end(); ++it2) {
                id = *it2;
                if (!God::instance()->my_rescheduleTarget[id]) God::instance()->my_target[id] = 0;
                God::instance()->my_rescheduleTarget[id] = true;
                if (fabs(God::instance()->my_target[id] - God::instance()->my_children_inarea[id]) <= 0.0001) {
                    continue;
                }
                if (areasensing[areaid] > 0) {
                    temp = left*God::instance()->my_childrensensing_inarea[id]/areasensing[areaid];
                }
                else {
                    temp = left*God::instance()->my_children_inarea[id]/areaalive[areaid];
                }
                if (temp < 0.005) temp = 0.005;
                if (God::instance()->my_target[id] + temp > God::instance()->my_children_inarea[id]) {  
                    temp = God::instance()->my_children_inarea[id] - God::instance()->my_target[id];
                }
                if (curTarget + temp > areareq) {
                    temp = areareq - curTarget;
                }
                God::instance()->my_target[id] += temp;
                curTarget += temp;
                printf("sink reschedule node %d target %f children %d curTarget %f temp %f @ %f\n",
                    id, God::instance()->my_target[id], God::instance()->my_children_inarea[id], curTarget, temp,NOW);
            }
            left = areareq - curTarget;
        }
        #endif
        #if 0
        double totallifetime = 0;
        for(vector<int>::iterator it2=(it->second).begin(); it2!=(it->second).end(); ++it2) {
            id = *it2;
            God::instance()->my_target[id] = 0;
            totallifetime += God::instance()->my_lifetime[id];
            God::instance()->my_rescheduleTarget[id] = true;
        }
        while(true) {
            for(vector<int>::iterator it2=(it->second).begin(); it2!=(it->second).end(); ++it2) {
                id = *it2;
                temp = left*God::instance()->my_lifetime[id]/totallifetime;
                if (temp < 0.001) temp = 0.001;
                if (God::instance()->my_target[id] + temp > God::instance()->my_children_inarea[id]) {  
                    temp = God::instance()->my_children_inarea[id] - God::instance()->my_target[id];
                }
                God::instance()->my_target[id] += temp;
                curTarget += temp;
            }
            left -= curTarget;
            curTarget = 0;
            if (left <= 0.0001) break;
        }
        #endif
    }
    if (scheduleMethod_ != 1) return;
    for(map< int, vector<int> >::iterator it=rescheduleDirectArea.begin(); it!=rescheduleDirectArea.end(); ++it) {
        onehopchildren.clear();
        onehopchildren.swap(it->second);
        printf("sink reschdule direct area node area %d size %u @ %f\n", it->first,(unsigned)onehopchildren.size(), NOW);
        JROSAllocate_multiple();
    }

}



