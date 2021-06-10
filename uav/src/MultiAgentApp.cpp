//  created:    2015/11/05
//  filename:   MultiAgentApp.cpp
//
//  author:     Guillaume Sanahuja
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    demo fleet
//
//
/*********************************************************************/

#include "MultiAgentApp.h"

#include "attQuatBkstp/attQuatBkstp.h"
#include "attQuatBkstp/attQuatBkstp_impl.h"

#include "zBackstepping/zBackstepping.h"
#include "zBackstepping/zBackstepping_impl.h"

#include "xyBackstepping/xyBackstepping.h"
#include "xyBackstepping/xyBackstepping_impl.h"

#include "xyFleet/xyFleet.h"
#include "xyFleet/xyFleet_impl.h"

#include "xyFleetPD/xyFleetPD.h"
#include "xyFleetPD/xyFleetPD_impl.h"

#include "xyFleetBkstp/xyFleetBkstp.h"
#include "xyFleetBkstp/xyFleetBkstp_impl.h"

#include "attSMC/attSMC.h"
#include "attSMC/attSMC_impl.h"

#include "xySMC/xySMC.h"
#include "xySMC/xySMC_impl.h"

#include "xyFleetSMC/xyFleetSMC.h"
#include "xyFleetSMC/xyFleetSMC_impl.h"




#include <TargetController.h>
#include <Uav.h>
#include <GridLayout.h>
#include <PushButton.h>
#include <DataPlot1D.h>
#include <Ahrs.h>
#include <MetaUsRangeFinder.h>
#include <MetaDualShock3.h>
#include <FrameworkManager.h>
#include <VrpnClient.h>
#include <MetaVrpnObject.h>
#include <TrajectoryGenerator2DCircle.h>
#include <Vector3D.h>
#include <Vector2D.h>
#include <PidThrust.h>
#include <Euler.h>
#include <Matrix.h>
#include <AhrsData.h>
#include <Ahrs.h>
#include <DoubleSpinBox.h>
#include <stdio.h>
#include <cmath>
#include <Tab.h>
#include <Pid.h>
#include <UdpSocket.h>
#include <string.h>
#include <sstream>
#include <vector>
#include <GroupBox.h>
#include <string>
#include <Label.h>

#include <TabWidget.h> 
#include <Quaternion.h>

#include <iostream>

#define PI ((float)3.14159265358979323846)

using namespace std;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::sensor;
using namespace flair::filter;
using namespace flair::meta;
using std::vector;


MultiAgentApp::MultiAgentApp(string broadcast,TargetController *controller): UavStateMachine(controller), behaviourMode(BehaviourMode_t::Default), vrpnLost(false) {
    Uav* uav=GetUav();
		
	VrpnClient* vrpnclient=new VrpnClient("vrpn", uav->GetDefaultVrpnAddress(),80);
	uavVrpn = new MetaVrpnObject(uav->ObjectName());
	getFrameworkManager()->AddDeviceToLog(uavVrpn);
	uav->GetAhrs()->YawPlot()->AddCurve(uavVrpn->State()->Element(2),DataPlot::Green);

    std::cout << "hola\n";
    std::cout << uav->ObjectName() << std::endl;
    Name_Drone = uav->ObjectName();
    std::cout << "El nombre del Drone es: " << Name_Drone << std::endl; 
    switch(Name_Drone[3]){
        case '0':
        {
            *ptrIdDrone = 0;
            cout << "El Id del Drone es: " << *ptrIdDrone << endl;
            break;
        }       
        case '1':
        {
            *ptrIdDrone = 1;
            cout << "El Id del Drone es: " << *ptrIdDrone << endl;          
            break;      
        }
        case '2':
        {
            *ptrIdDrone = 2;
            cout << "El Id del Drone es: " << *ptrIdDrone << endl;
            break;
        }
    }


        // En esta sección se crean los demás drones y se les asigna un nombre
    for(int i=0;i<3;i++) {
        stringstream s;
        s << "x4_" << i;
        //cout << s << endl;
        cout << s.str();   // Imprime la cadena del nuevo nombre del drone que está creando 
        if(s.str()==uav->ObjectName())
        {
            ArrayDrones[i] = uavVrpn;//self
            cout << " => if - Se ha creado el drone: " << s.str() << endl; 
        }
        else
        {
            ArrayDrones[i] = new MetaVrpnObject(s.str());//others
            cout << " => else - Se ha creado el drone: " << s.str() << endl; 
            //uavVrpn->XyPlot()->AddCurve(ArrayDrones[i]->Output()->Element(5,0),ArrayDrones[i]->Output()->Element(4,0));
        }
        //cout << i << endl;
    }

    vrpnclient->Start();
    HomePB = new PushButton(GetButtonsLayout()->NewRow(),"Home");
    FormationPB = new PushButton(GetButtonsLayout()->LastRowLastCol(),"Hold_Pos / Formation");

    x_ref_spb=new DoubleSpinBox(GetButtonsLayout()->NewRow(),"Home Position(x)"," m",-5,5,0.1,1,0);
    y_ref_spb=new DoubleSpinBox(GetButtonsLayout()->LastRowLastCol(),"Home Position(y)"," m",-5,5,0.1,1,0);
    z_ref_spb=new DoubleSpinBox(GetButtonsLayout()->LastRowLastCol(),"Home Position(z)"," m",-5,5,0.1,1,0);

    // wtih this we create an horizontal tab at the top of the windowF
    Tab *QuadCtrl = new Tab(getFrameworkManager()->GetTabWidget(),"Quad Tab");

    // with this we create three vertical tabs 
    TabWidget *ConfWidget = new TabWidget(QuadCtrl->NewRow(),"User Control");
    GainsTab = new Tab(ConfWidget,"Setup");
    AnglesGraph = new Tab(ConfWidget,"Attitude Graphs");
    PositionGraph = new Tab(ConfWidget,"Position Graphs");


    Tab *MultiAgent_tab = new Tab(getFrameworkManager()->GetTabWidget(),"MultiAgent");
    // with this we create three vertical tabs 
    TabWidget *tab_widget_MultiAgent = new TabWidget(MultiAgent_tab->NewRow(),"User Control");
    setup_myTab_MA = new Tab(tab_widget_MultiAgent,"MultiAgent");
    graph_law_angles_MA = new Tab(tab_widget_MultiAgent,"Graphs 01 pf");
    graph_law_position_MA = new Tab(tab_widget_MultiAgent,"Graphs 02 pf");



    //xCtrl = new xyBackstepping(GainsTab->At(0,0),"xCtrl-Backstepping");
    //xCtrl->UseDefaultPlot(PositionGraph->LastRowLastCol());

    xCtrlSMC = new xySMC(GainsTab->At(0,0),"xCtrl-SMC");
    xCtrlSMC->UseDefaultPlot(PositionGraph->NewRow());
    

    //yCtrl = new xyBackstepping(GainsTab->At(0,1),"yCtrl-Backstepping");
    //yCtrl->UseDefaultPlot(PositionGraph->LastRowLastCol());

    yCtrlSMC = new xySMC(GainsTab->At(0,1), "yCtrl - SMC");
    yCtrlSMC->UseDefaultPlot(PositionGraph->LastRowLastCol());

    z_Ctrl = new zBackstepping(GainsTab->At(0,2),"zCtrl");
    z_Ctrl->UseDefaultPlot(PositionGraph->LastRowLastCol());

    //xFormation = new xyFleet(setup_myTab_MA->At(0,0),"x_Formation");
    //xFormation->UseDefaultPlot(graph_law_angles_MA->NewRow());

    //xFormBkstp = new xyFleetBkstp(setup_myTab_MA->At(0,0),"x_Formation");
    //xFormBkstp->UseDefaultPlot(graph_law_angles_MA->NewRow());

    xFormSMC = new xyFleetSMC(setup_myTab_MA->At(0,0),"x_Formation");
    xFormSMC->UseDefaultPlot(graph_law_angles_MA->NewRow());



    //yFormation = new xyFleet(setup_myTab_MA->At(1,0),"y_Formation");
    //yFormation->UseDefaultPlot(graph_law_angles_MA->LastRowLastCol());

    //yFormBkstp = new xyFleetBkstp(setup_myTab_MA->At(1,0),"y_Formation");
    //yFormBkstp->UseDefaultPlot(graph_law_angles_MA->LastRowLastCol());

    yFormSMC = new xyFleetSMC(setup_myTab_MA->At(1,0),"y_Formation");
    yFormSMC->UseDefaultPlot(graph_law_angles_MA->NewRow());





    //attQuat = new attQuatBkstp(setup_myTab->At(1,1),"Attitude Quaternion");
    //attQuat->UseDefaultPlot(graph_law_angles->LastRowLastCol());

    attQuatSMC = new  attSMC(GainsTab->At(3,1),"Attitude Quaternion- SMC");
    attQuatSMC->UseDefaultPlot(AnglesGraph->LastRowLastCol());


    getFrameworkManager()->AddDeviceToLog(z_Ctrl);
    //getFrameworkManager()->AddDeviceToLog(xCtrl);
    //getFrameworkManager()->AddDeviceToLog(yCtrl);
    //getFrameworkManager()->AddDeviceToLog(xFormBkstp);
    //getFrameworkManager()->AddDeviceToLog(yFormBkstp);
    //getFrameworkManager()->AddDeviceToLog(attQuat);
    getFrameworkManager()->AddDeviceToLog(attQuatSMC);
    getFrameworkManager()->AddDeviceToLog(xCtrlSMC);
    getFrameworkManager()->AddDeviceToLog(yCtrlSMC);

    getFrameworkManager()->AddDeviceToLog(xFormSMC);
    getFrameworkManager()->AddDeviceToLog(yFormSMC);



    GroupBox *RelDistx1 = new GroupBox(setup_myTab_MA->At(0,1), "Rel Dist x F1");
    for(int i = 0; i < 3 ; i++){
        for(int j = 0; j < 3; j++){
        std::string BoxName = "F1x["+std::to_string(i)+"]["+std::to_string(j)+"]:";
        F1x[i][j] = new DoubleSpinBox(RelDistx1->At(i,j),BoxName," m",-10,10,0.1,1,0);
        }
    }

    GroupBox *RelDistx2 = new GroupBox(setup_myTab_MA->At(0,2), "Rel Dist x F2");
    for(int i = 0; i < 3 ; i++){
        for(int j = 0; j < 3; j++){
            std::string BoxName = "F2x["+std::to_string(i)+"]["+std::to_string(j)+"]:";
            F2x[i][j] = new DoubleSpinBox(RelDistx2->At(i,j),BoxName," m",-10,10,0.1,1,0);
        }
    }

    GroupBox *RelDisty1 = new GroupBox(setup_myTab_MA->At(1,1), "Rel Dist y F1");
    for(int i = 0; i < 3 ; i++){
        for(int j = 0; j < 3; j++){
        std::string BoxName = "F1y["+std::to_string(i)+"]["+std::to_string(j)+"]:";
        F1y[i][j] = new DoubleSpinBox(RelDisty1->At(i,j),BoxName," m",-10,10,0.1,1,0);
        }
    }

    GroupBox *RelDisty2 = new GroupBox(setup_myTab_MA->At(1,2), "Rel Dist y F2");
    for(int i = 0; i < 3 ; i++){
        for(int j = 0; j < 3; j++){
            std::string BoxName = "F2y["+std::to_string(i)+"]["+std::to_string(j)+"]:";
            F2y[i][j] = new DoubleSpinBox(RelDisty2->At(i,j),BoxName," m",-5,5,0.1,1,0);
        }
    }

    GroupBox *F1form = new GroupBox(setup_myTab_MA->At(3,1),"Formation 1 Selection");
    f1PB = new PushButton(F1form->NewRow(),"Formation 01");
    LandPB1 = new PushButton(F1form->NewRow(),"Emergency Land");



    GroupBox *F2form = new GroupBox(setup_myTab_MA->At(3,2),"Formation 2 Selection");
    f2PB = new PushButton(F2form->NewRow(),"Formation 02");
    LandPB2 = new PushButton(F2form->NewRow(),"Emergency Land");


    GroupBox *Transition = new GroupBox(setup_myTab_MA->At(3,0),"Transition");
    HomePB_01 = new PushButton(Transition->NewRow(),"Home");
    FormationPB_01 = new PushButton(Transition->NewRow(),"Hold_Pos / Formation");

    GroupBox *RefFleet = new GroupBox(setup_myTab_MA->At(4,0),"Change Fleet Position");
    x_Fleet=new DoubleSpinBox(RefFleet->NewRow(),"Fleet Position(x)"," m",-5,5,0.1,1,0);
    y_Fleet=new DoubleSpinBox(RefFleet->LastRowLastCol(),"Fleet Position(y)"," m",-5,5,0.1,1,0);
    btnUpdateFleetPos = new PushButton(RefFleet->NewRow(),"Update Fleet Position");


    GroupBox *PosFleet = new GroupBox(setup_myTab_MA->At(4,1),"Actual Fleet Position");
    HomePosX = new Label(PosFleet->NewRow(),"HomeX");
    HomePosY = new Label(PosFleet->LastRowLastCol(),"HomeY");

    HomePosX->SetText("Pos Fleet X: %f",x_Fleet->Value());
    HomePosY->SetText("Pos Fleet Y: %f",y_Fleet->Value());
    

    //xGainLead = new DoubleSpinBox(RefFleet->NewRow(),"Gain Fleet x"," m",-5,5,0.1,1,0);
    //yGainLead = new DoubleSpinBox(RefFleet->NewRow(),"Gain Fleet y"," m",-5,5,0.1,1,0);



    message=new UdpSocket(uav,"Message",broadcast,true);

    customReferenceOrientation= new AhrsData(this,"reference");
    uav->GetAhrs()->AddPlot(customReferenceOrientation,DataPlot::Yellow);
    AddDataToControlLawLog(customReferenceOrientation);

    customOrientation=new AhrsData(this,"orientation");

}

MultiAgentApp::~MultiAgentApp() {
}

/****************************************************************
        ATTTIUDE EXTRACTION FROM DRONE
*****************************************************************
    -> Extract the attitude from the drone the measurements are obtained from the IMU  and optitrack
    -> The output is given in quaternion for attitude
    -> The output is given in 3D vector for angular speed.
*****************************************************************/
const AhrsData *MultiAgentApp::GetOrientation(void) const {
    //get yaw from vrpn
    // Generates a quaternion object named vrpnQuaternion
    Quaternion vrpnQuaternion;
    //GetQuaternion works with refernces, hence the vrpnQuaternion is filled with the 
    //uav attitude in quaternions
    uavVrpn->GetQuaternion(vrpnQuaternion);

    // get roll, pitch and w from imu
    // Generates a quaternion object named ahrsQuaternion
    Quaternion ahrsQuaternion;
    // Generates a float Vector3D named ahrsAngularSpeed
    Vector3Df ahrsAngularSpeed;
    //GetQuaternionAngularRates works with references,
    //GetDefaultOrientation works with references
    //ahrsQuaternion is filled 
    //ahrsAngular Speed is filled
    GetDefaultOrientation()->GetQuaternionAndAngularRates(ahrsQuaternion, ahrsAngularSpeed);

    //an object ahrsEuler  of type Euler (3D vector with roll, pitch and yaw components) is created
    //The object arhrsEuler is filled with the Euler Angles associated to the Quadrotor Quaternion attitude
    Euler ahrsEuler=ahrsQuaternion.ToEuler();
    //Once the ahrsEuler is filled with the attitude quaternion (data obtained from IMU)
    // the yaw angle is changed by the measure from the vrpn() (Optitrack object)
    // this is done to avoid the derive of the angle, (Error usually presented on the IMU)
    ahrsEuler.yaw=vrpnQuaternion.ToEuler().yaw;
    // A Quaternion object named mixQuaternion is obtained using the Euler angles
    // roll->IMU            pitch->IMU          yaw->Optitrack
    Quaternion mixQuaternion=ahrsEuler.ToQuaternion();

    //custom orientation is filled with the mixQuaternion and the AngularSpeed 
    // Observe that Angular Speed is from the IMU (roll, pitch and yaw)
    // Why didn't they use the angular speed also from the Optitrack????
    customOrientation->SetQuaternionAndAngularRates(mixQuaternion,ahrsAngularSpeed);

    return customOrientation;
}

/******************************************************
        ALTITUDE VALUES   (FROM OPTITRACK)
*******************************************************
    ->Extract the altitude from the optitrack
    ->Changes into the Drone's frame
******************************************************/
void MultiAgentApp::AltitudeValues(float &z,float &dz) const{
    // a float 3DVector is created to store the uav_pos
    // a float 3DVector is created to store the uav_vel
    Vector3Df uav_pos,uav_vel;

    //GetPosition and GetSpeed work with references
    // uav_pos is filled with the uav position from Optitrack
    // uav_vel is filled with the uav velocity from Optitrack
    uavVrpn->GetPosition(uav_pos);
    uavVrpn->GetSpeed(uav_vel);
    //z and dz must be in uav's frame
    z=-uav_pos.z;
    dz=-uav_vel.z;
}



void MultiAgentApp::ExtraCheckPushButton(void) {
    CheckMessages();
    if((HomePB->Clicked()||HomePB_01->Clicked()) && (behaviourMode==BehaviourMode_t::Default||behaviourMode==BehaviourMode_t::Formation)){
        //formationSel = FormationSel_t::NoFormation;
        Form_index = 0;
        GotoHome();
    }
    if((FormationPB->Clicked()||FormationPB_01->Clicked()) && (behaviourMode==BehaviourMode_t::Homing || behaviourMode==BehaviourMode_t::Formation)){
        Form_index = 0;
        message->SendMessage("Formation");
        SetHoldPosition();
        FlightFormation();
    }
    if(f1PB->Clicked() && (behaviourMode==BehaviourMode_t::Formation)){
        //FlightFormation();
        message->SendMessage("Formation1");
        Printf("Formation1 clicked\n");
        Form_index = 1;
    }

    if(f2PB->Clicked() && (behaviourMode==BehaviourMode_t::Formation)){
        //FlightFormation();
        message->SendMessage("Formation2");
        Form_index = 2;
    }

    if(LandPB2->Clicked() || LandPB1->Clicked()){
        message->SendMessage("Landing");
        Land();

    }
    if(btnUpdateFleetPos->Clicked()){     
        string msgStr = "FP,"+to_string(x_Fleet->Value())+","+to_string(y_Fleet->Value());
        //cout << *msgStr << endl; 
        Printf("send changing position\n");
        message->SendMessage(msgStr);
        LeaderPosX = x_Fleet->Value();
        LeaderPosY = y_Fleet->Value();
        UpdatePositionLabel();
    }


}


void MultiAgentApp::SignalEvent(Event_t event) {
    UavStateMachine::SignalEvent(event);
    CheckMessages();
    switch(event) {
    case Event_t::TakingOff:
        behaviourMode=BehaviourMode_t::Default;
        message->SendMessage("TakeOff");
        vrpnLost=false;
        break;

    case Event_t::EmergencyStop:
        message->SendMessage("EmergencyStop");
        break;

    case Event_t::StartLanding:
        message->SendMessage("Landing");
        break;
    }
}


void MultiAgentApp::CheckMessages(void) {
    char msg[64];
    char src[64];
    size_t src_size=sizeof(src);
    while(message->RecvMessage(msg,sizeof(msg),TIME_NONBLOCK,src,&src_size)>0) {
        //printf("%s %s\n",GetUav()->ObjectName().c_str(),src);
        if(strcmp(src,GetUav()->ObjectName().c_str())!=0) {

            if(strcmp(msg,"TakeOff")==0) {
                Printf("TakeOff fleet\n");
                TakeOff();
            }
            if(strcmp(msg,"Landing")==0) {
                Printf("Landing fleet\n");
                Land();
            }
            if(strcmp(msg,"EmergencyStop")==0) {
                Printf("EmergencyStop fleet\n");
                EmergencyStop();
            }
            if(strcmp(msg,"Homing")==0){
                //Printf("Homing the fleet\n");
                GotoHome();
            }

            if(strcmp(msg,"Formation")==0){
                SetHoldPosition();
                Printf("Consensus\n");
                FlightFormation();
                Form_index = 0;
            }

            if(strcmp(msg,"Formation1")==0){
                Printf("Formation1 receive\n");
                Form_index = 1;
            
            }

            if(strcmp(msg,"Formation2")==0){
                Printf("Formation2\n");
                Form_index = 2;


            }
            if(msg[0]=='F' && msg[1] =='P'){
                Printf("receive Changing Formation Position\n");
                std::string posmsg(msg);
                //cout << posmsg << endl; 
                ParseFleetPositionMessage(posmsg);
                
                //cout << "Parsing ok" << endl;

            }
        }
    }
}

void MultiAgentApp::ExtraSecurityCheck(void) {
    if (!vrpnLost && behaviourMode!=BehaviourMode_t::Default) {
        if (!uavVrpn->IsTracked(500)) {
            Thread::Err("Optitrack, uav lost\n");
            vrpnLost=true;
            EnterFailSafeMode();
            Land();
        }
    }
}

void MultiAgentApp::GotoHome(void){
    //std::cout << "Going Home" << endl; 
    auto _TorqueMode = GetTorqueMode();
    auto _OrientationMode = GetOrientationMode();
    auto _ThrustMode = GetThrustMode();

    if(_OrientationMode == OrientationMode_t::Custom)
        std::cout << "OrientationMode: CUSTOM\t"; 
    else if (SetOrientationMode(OrientationMode_t::Custom))
        std:: cout << "OrientationMode: CUSTOM\t";
    else
        std::cout << "Error: OrientationMode: DEFAULT\t";
    

    if(_TorqueMode == TorqueMode_t::Custom)
        std::cout << "TorqueMode: CUSTOM\t";
    else if (SetTorqueMode(TorqueMode_t::Custom))
        std::cout << "TorqueMode: CUSTOM\t";
    else
        std::cout << "Error - TorqueMode: Default\t";

    if(_ThrustMode == ThrustMode_t::Custom)
        std::cout << "AltitudeMode: CUSTOM\n";
    else if (SetThrustMode(ThrustMode_t::Custom))
        std::cout << "AltitudeMode: CUSTOM\n";
    else
        std::cout << "Error: AltitudeMode: DEFAULT\n";

    _TorqueMode = GetTorqueMode();
    _OrientationMode = GetOrientationMode();
    _ThrustMode = GetThrustMode();

    if((behaviourMode!=BehaviourMode_t::Homing) && (_OrientationMode==OrientationMode_t::Custom) &&
        (_TorqueMode==TorqueMode_t::Custom) && (_ThrustMode==ThrustMode_t::Custom)){
        std::cout << "All conditions to Home OK -> " << std::endl; 
        behaviourMode=BehaviourMode_t::Homing;
        message->SendMessage("Homing");
        }
}


void MultiAgentApp::FlightFormation(void){
    //std::cout << "Going Home" << endl; 
    Form_index = 0;
    
    if(behaviourMode==BehaviourMode_t::Homing){
        //cout << "Going Home" << endl;
        behaviourMode=BehaviourMode_t::Formation;
        message->SendMessage("Formation");
        cout << "Formación iniciada" << endl;
    }
    else if( behaviourMode == BehaviourMode_t::Formation)
        std::cout << "Ya está en modo FORMACION " << std::endl; 
     
}


/*********************************************************************************************************************
                        ComputeCustomTorques
**********************************************************************************************************************
This method has to be overloaded in order to compute the attitude control
The method woks with references, it receives a reference of  torque, hence, the the main objective of this method
is to fill the torque Euler object with the control for roll, pitch and yaw.
torques.roll = ctrsl_roll
torques.pitch = ctrl_pitch
torques.yaw = ctrl_yaw
**********************************************************************************************************************/
void MultiAgentApp::ComputeCustomTorques(Euler &torques){
  // REFERENCES in Quaternion
  Quaternion refQuaternion;
  Vector3Df refAngularRates;
  // Extract the ReferenceQuaternion
  GetReferenceQuaternion(refQuaternion, refAngularRates);


  // Quaternion and 3D vector for the currentQuaternion(attitude)
  // and the angularRates(omega)
  Quaternion currentQuaternion;
  Vector3Df currentAngularRates;
  // Extract the UAV Orientation in quaternions and the angular Rates
  GetUAVOrientation(currentQuaternion, currentAngularRates);

/*
  attQuat->SetValues(currentQuaternion.q0,currentQuaternion.q1,currentQuaternion.q2,currentQuaternion.q3,
                     refQuaternion.q0, refQuaternion.q1, refQuaternion.q2, refQuaternion.q3,
                     currentAngularRates.x, currentAngularRates.y, currentAngularRates.z); 
  attQuat->Update(GetTime());

  torques.roll  = attQuat->Output(0);//0.0;
  torques.pitch = attQuat->Output(1);//-attQuat->Output(1);//u_pitch->Output();
  torques.yaw   = attQuat->Output(2);//u_yaw->Output();
*/


  attQuatSMC->SetValues(currentQuaternion.q0,currentQuaternion.q1,currentQuaternion.q2,currentQuaternion.q3,
                     refQuaternion.q0, refQuaternion.q1, refQuaternion.q2, refQuaternion.q3,
                     currentAngularRates.x, currentAngularRates.y, currentAngularRates.z); 
  attQuatSMC->Update(GetTime());

  torques.roll  = attQuatSMC->Output(0);//0.0;
  torques.pitch = attQuatSMC->Output(1);//-attQuat->Output(1);//u_pitch->Output();
  torques.yaw   = attQuatSMC->Output(2);//u_yaw->Output();
}

/******************************************************************************************************
                    ComputeCustomThrust
********************************************************************************************************
This is the implmentation of the altitude control law
INPUT -> None
OUTPUT -> float - the thurst computed

*****************************************************************************************************/
float MultiAgentApp::ComputeCustomThrust(void){


    Vector3Df PositionCtrl;
    ComputePositionControllers(PositionCtrl);

    float Thrust = PositionCtrl.z;

    return Thrust;
}


/*******************************************************************************
        EXTRACT THE ATITTUDE REFERENCE (desired quaternion  q_d)

        HERE IS THE POSITION CONTROL
********************************************************************************
    -> Extract the references for position
    -> Computes the tracking error (e)
    -> Computes the velocity trackin errror (dot_e)
    -> Sends the e and dot_e to the pid clas for x and y
    -> Once the control law is computed, it is taken as reference to the attitude
    -> Here we can change the attitude selection, from the Dualshock or from the posision control.
            if(position_mode){
                refAngles.roll = uX->Output();
                refAngles.pitch = uY->Output();
                refAngles.yaw = 0. // this can be changed
            else{
                refAngles.roll = measure_from_Dualshock.x
                refAngles.pitch = measure_from_Dualshock.y
                refAngles.yaw = measure_from_Dualshock.z
                }

    -> NO INPUT
    -> OUTPUT AhrsData  
    
    THIS METHOD HAS TO BE OVERLOADED IN ORDER TO IMPLMENT YOUR CONTROL LAW
*******************************************************************************/
AhrsData *MultiAgentApp::GetReferenceOrientation(void) {

    Quaternion Qd;
    ComputeDesiredQuaternion(Qd);

    customReferenceOrientation->SetQuaternionAndAngularRates(Qd,Vector3Df(0,0,0));

    // Returns the Attitude reference in quaternion and Omeg_vec = [ 0 0 0 ] ^T
    return customReferenceOrientation;
}


// This is to obtain the reference quaternion does not matter if is from Joystick or position control
void MultiAgentApp::GetReferenceQuaternion(flair::core::Quaternion &refQuat, flair::core::Vector3Df &refOmega){
    if(GetOrientationMode() == OrientationMode_t::Manual){
    const AhrsData *refOrientation = GetDefaultReferenceOrientation();
    refOrientation->GetQuaternionAndAngularRates(refQuat, refOmega);
    //cout << "JoyMode" << endl;
    }
  else{
    const AhrsData *refOrientation = GetReferenceOrientation();
    refOrientation->GetQuaternionAndAngularRates(refQuat, refOmega);
    //cout << "PosCtrl   " << endl; 
    } 
} 

void MultiAgentApp::GetUAVOrientation(flair::core::Quaternion &uavQuaternion, flair::core::Vector3Df &uavAngSpeed){
    if(GetOrientationMode() == OrientationMode_t::Manual){
        GetDefaultOrientation()->GetQuaternionAndAngularRates(uavQuaternion, uavAngSpeed);
    }
  else{
      const AhrsData *currentOrientation = GetOrientation();
      currentOrientation->GetQuaternionAndAngularRates(uavQuaternion,uavAngSpeed);
    } 
}


void MultiAgentApp::SetHoldPosition(){

    uavVrpn->GetPosition(Pos2Hold);

    }


void MultiAgentApp::ComputePositionControllers(flair::core::Vector3Df &Controller){

    float ux, uy, uz;

    Vector3Df uav_pos,uav_vel;

    uavVrpn->GetPosition(uav_pos);
    uavVrpn->GetSpeed(uav_vel);

    // two scalars are declared to store the position and velocity of the drone along the z-axis.
    float pos_z;
    float vel_z;

    // AltitudeValues works with references, hence, the variagles pos_z and vel_z are filled within this
    // function
    AltitudeValues(pos_z, vel_z);

    // A random reference for altitude, this can be changed to use the dualshock measure, we don't know
    // how but it can be done, we should check the UavStateMachine classs, everythint is in there.
    float reference_altitude = 1.30;

    float dot_error = vel_z;
    z_Ctrl->SetValues(pos_z, reference_altitude,vel_z,0.0);
    z_Ctrl->Update(GetTime());
    uz = -z_Ctrl->Output();

    if(behaviourMode==BehaviourMode_t::Homing || Form_index == 0){
        float home_x = x_ref_spb->Value();
        float home_y = y_ref_spb->Value();
        /*
        xCtrl->SetValues(uav_pos.x,home_x,uav_vel.x,0.0);
        xCtrl->Update(GetTime());

        yCtrl->SetValues(uav_pos.y,home_y,uav_vel.y,0.0);
        yCtrl->Update(GetTime());

        ux = xCtrl->Output();
        uy = yCtrl->Output();
        */


        xCtrlSMC->SetValues(uav_pos.x,home_x,uav_vel.x,0.0);
        xCtrlSMC->Update(GetTime());

        yCtrlSMC->SetValues(uav_pos.y,home_y,uav_vel.y,0.0);
        yCtrlSMC->Update(GetTime());

        ux = xCtrlSMC->Output();
        uy = yCtrlSMC->Output();


    }

    if(behaviourMode==BehaviourMode_t::Formation && Form_index == 0){
        /*
        xCtrl->SetValues(uav_pos.x,Pos2Hold.x,uav_vel.x,0.0);
        xCtrl->Update(GetTime());

        yCtrl->SetValues(uav_pos.y,Pos2Hold.y,uav_vel.y,0.0);
        yCtrl->Update(GetTime());

        ux = xCtrl->Output();
        uy = yCtrl->Output();
        */  

        xCtrlSMC->SetValues(uav_pos.x,Pos2Hold.x,uav_vel.x,0.0);
        xCtrlSMC->Update(GetTime());

        yCtrlSMC->SetValues(uav_pos.y,Pos2Hold.y,uav_vel.y,0.0);
        yCtrlSMC->Update(GetTime());

        ux = xCtrlSMC->Output();
        uy = yCtrlSMC->Output();
        
    }



    if(behaviourMode==BehaviourMode_t::Formation && Form_index >= 1){

        std::vector<float> Fleet_x{0,0,0};
        std::vector<float> Fleet_y{0,0,0};

        std::vector<float> RelDistX{0,0,0};
        std::vector<float> RelDistY{0,0,0};

        float XFleet = LeaderPosX;
        float YFleet = LeaderPosY;

        //UpdatePositionLabel();
        //cout << "XFleet: " << XFleet << "    YFleet:" << YFleet << endl; 

        //MULTIAGENT PROTOCOL
        FleetPos(Fleet_x, Fleet_y);
        ExtractRelativeDistances(IdDrone,RelDistX,RelDistY);
        float XMultiagentCtrl;
        float YMultiagentCtrl;

        /*
        xFormBkstp->SetValues(IdDrone,Fleet_x,RelDistX,uav_vel.x,0.0,XFleet);
        xFormBkstp->Update(GetTime());
        XMultiagentCtrl = xFormBkstp->Output();

        yFormBkstp->SetValues(IdDrone,Fleet_y,RelDistY,uav_vel.y,0.0,YFleet);
        yFormBkstp->Update(GetTime());
        YMultiagentCtrl = yFormBkstp->Output();
        */


        xFormSMC->SetValues(IdDrone,Fleet_x,RelDistX,uav_vel.x,0.0,XFleet);
        xFormSMC->Update(GetTime());
        XMultiagentCtrl = xFormSMC->Output();

        yFormSMC->SetValues(IdDrone,Fleet_y,RelDistY,uav_vel.y,0.0,YFleet);
        yFormSMC->Update(GetTime());
        YMultiagentCtrl = yFormSMC->Output();



        ux = XMultiagentCtrl;
        uy = YMultiagentCtrl;





    }

    Controller.x = ux;
    Controller.y = uy;
    Controller.z = uz;
}


void MultiAgentApp::ComputeDesiredQuaternion(flair::core::Quaternion &DesiredQuaternion){
    Vector3Df UPos;
    ComputePositionControllers(UPos);

    UPos.Normalize();

    // This vector can be changed... since the z-axis is pointing in a different direction
    Vector3Df v(0.0,0.0,1.0);

    float qd_0 = sqrt((1 + DotProduct(UPos,v))/2);

    Vector3Df q_vec = VectorialProduct(UPos,v);

    DesiredQuaternion.q0 = qd_0;
    DesiredQuaternion.q1 = qd_0*q_vec.x;
    DesiredQuaternion.q2 = qd_0*q_vec.y;
    DesiredQuaternion.q3 = qd_0*q_vec.z;

    DesiredQuaternion.Normalize();
}


float MultiAgentApp::DotProduct(flair::core::Vector3Df &v1, flair::core::Vector3Df &v2){
    return (v1.x*v2.x + v1.y*v2.y + v1.z*v2.z);
}


flair::core::Vector3Df MultiAgentApp::VectorialProduct(flair::core::Vector3Df &m, flair::core::Vector3Df &v){
   return(Vector3Df(m.y*v.z - v.y*m.z,
                    v.x*m.z - m.x*v.z,
                    m.x*v.y - v.x*m.y));

    }

void MultiAgentApp::FleetPos(std::vector<float> &fleet_x, std::vector<float> &fleet_y){
    Vector3Df genericPos;
    
    for(int i = 0 ; i < 3 ; i++){
        ArrayDrones[i]->GetPosition(genericPos);
        fleet_x[i] = genericPos.x;
        fleet_y[i] = genericPos.y;
    }

}

void MultiAgentApp::ExtractRelativeDistances(int idDrone, std::vector<float> &reldistx, std::vector<float> &reldisty){
    
   if (Form_index == 1){
    for(int i = 0; i < 3; i++){
        reldistx[i] = F1x[idDrone][i]->Value();
        reldisty[i] = F1y[idDrone][i]->Value();
    }

   }

    if (Form_index == 2){        
        for(int i = 0; i < 3; i++){
            reldistx[i] = F2x[idDrone][i]->Value();
            reldisty[i] = F2y[idDrone][i]->Value();
        }
        
    }

    if (Form_index == 0){        
        for(int i = 0; i < 3; i++){
            reldistx[i] = 0.0;
            reldisty[i] = 0.0;
        }
    }
   
}

void MultiAgentApp::ParseFleetPositionMessage(std::string &ParsingStr){
    std::istringstream ss(ParsingStr);
    std::string splitstr;
    std::vector<std::string> splitVector;

    while(std::getline(ss,splitstr,',')){
        splitVector.push_back(splitstr);
    }

    //cout << "Printing the vector" << endl; 
   // cout << splitVector[0] << endl; 
    //cout << splitVector[1]  << endl; 
    //cout << splitVector[2]  << endl; 
    this->LeaderPosX = stof(splitVector[1]);
    this->LeaderPosY = stof(splitVector[2]);
    //cout << "LeaderX: " << LeaderPosX << "    LeaderY: " << LeaderPosY; 

    //cout << "   " << *StrX << "   " << *StrY << endl;

    UpdatePositionLabel();
}

void MultiAgentApp::UpdatePositionLabel(){
    HomePosX->SetText("Pos Fleet X: %f",LeaderPosX);
    HomePosY->SetText("Pos Fleet Y: %f",LeaderPosY);
}



