/*
 * rilapplication.h
 *
 *  Created on: Aug 19, 2009
 *      Author: yang
 */

#ifndef RILAPP_H_
#define RILAPP_H_

#include "rilcommon.h"
#include <vector>

class RILAppTimer : public TimerHandler {
public:
	RILAppTimer(RILApp* app, int tp) : appinstance_(app), type(tp), continueRunning(true){}
	~RILAppTimer(){
        appinstance_ = 0;
    }
	void stoptimer() {continueRunning = false;}
protected:
	void expire(Event*);
private:
	RILApp* appinstance_;
	int type;
	bool continueRunning;
};

class RILApp : public Application{
public:
	RILApp(int nodeid);
	~RILApp();
	void 	timeout(int timertype);                 // timeout function called by nrr timer
	void 	process_data(int size, AppData* data);  // Process incoming data
    void    UpdateChildrenInfo();
    void    FixAllocate();
    void    BalanceAllocate();
    void    JROSAllocate_single();
    void    JROSAllocate_multiple();
    void    UpdateAndSchedule();
    void    VerifyInitialization();
    void    SinkMonitor(double nodereq);
protected:
	int 	command(int argc, const char*const* argv);
	void 	start();
	void 	stop();
private:
	// function members
	void 	recvMsg(RILAppMessageT_* msg);          // recv data in msg format
	void 	sendMsg(RILAppMessageT_* msg);
    void    dumpMsg(RILAppMessageT_* msg, bool yes);
    void    produceMsg(RILAppMessageT_* msg, int msgtype);
    double  next_interval();
	RILAppTimer* dataT_;                            // timer for data transmitting
    RILAppTimer* controlT_;                         // timer to update schedule
    int     myID;
    int     packetseq;
    double  dataInterval_;
    double  sensingReq_;
    double  fixedPercent_;
    int     scheduleMethod_;
    std::vector<int> onehopchildren;
    double  nextsendtime;
    double  fixcost;// fixed cost per second
    double  txcost;   // tx cost per second given 100% duty cycle sensing activities
    double  rxcost;   // rx cost per second given 100% duty cycle sensing activities
    double  sensingcost;  // sensing cost per second given 100% duty cycle sensing activities
    double  periodsensing;
};

#endif /* NRRAPPLICATION_H_ */



