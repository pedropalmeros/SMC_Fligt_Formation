//  created:    2015/11/05
//  filename:   MultiAgentApp.h
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

#ifndef MULTIAGENTAPP_H
#define MULTIAGENTAPP_H

#include <UavStateMachine.h>
#include <vector>
#include <Vector3D.h>


namespace flair {
    namespace core {
        class FrameworkManager;
        class UdpSocket;
        class AhrsData;
    }
    namespace filter {
        class TrajectoryGenerator2DCircle;
        class attQuatBkstp;
        class xyBackstepping;
        class zBackstepping;
        class xyFleet;
        class xyFleetPD;
        class xyFleetBkstp;
        class attSMC;
        class xySMC;
        class xyFleetSMC;
    }
		namespace meta {
        class MetaVrpnObject;
    }
    namespace gui {
        class DoubleSpinBox;
        class Label;
    }
}


class MultiAgentApp : public flair::meta::UavStateMachine {
    public:
        MultiAgentApp(std::string broadcast,flair::sensor::TargetController *controller);
        ~MultiAgentApp();

    private:
        enum class BehaviourMode_t {
            Default,
            Homing,
            Formation,
        };

        BehaviourMode_t behaviourMode;
        bool vrpnLost;

        /*************************************************************************
        ****       Computes the mixed orientation
        **************************************************************************
        Computes the Quad attitude mixing the data from the Optitrack and IMU
        **************************************************************************/
        const flair::core::AhrsData *GetOrientation(void) const;
        
        /*************************************************************************
        ****       Computes the altitude 
        **************************************************************************
        Computes the altitude from the optitrack
        **************************************************************************/      
        void AltitudeValues(float &z,float &dz) const;

        /**************************************************************************
        *****       pushButthon from GUI
        ***************************************************************************
        Checks which button has been pressed from the GUI
        ****************************************************************************/
        void ExtraCheckPushButton(void);

        /****************************************************************************
        ***             SAFETY OF THE DRONE
        *****************************************************************************
        SignalEvent
        ExtraSecurityCheck
        ****************************************************************************/
        void SignalEvent(Event_t event);
        void ExtraSecurityCheck(void);

        /*************************************************************************
        ***     CheckMessages - MultiAgent
        ***********************************************************************
        check the messages from other drones to take an action
        ***********************************************************************/
        void CheckMessages(void);


        /**********************************************************************
        ***         GotoHome
        ***********************************************************************
        Sends each drone to its home position
        ***********************************************************************/
        void GotoHome(void);

        /**********************************************************************
        ***         FlightFormation
        ***********************************************************************
        Sends each drone to its home position
        ***********************************************************************/
        void FlightFormation(void);


        /***********************************************************************
                GetReferenceOrientation
        ************************************************************************
        Computes the Orientation from the position Cointrol
        ***********************************************************************/
        flair::core::AhrsData *GetReferenceOrientation(void);


        /**********************************************************************************************
        ***  virtual void flair::meta::UavStateMachine::ComputeCustomTorques(flair::core::Euler&)   ***
        ***********************************************************************************************
         If you want to read the signal from the joystick and set your attitude controller you need
         to override this function, without it, you won't be able to set the tores to your custom

        This funciton accepts a torque object of type Euler, but it works with references, hence in this
        method you will fill the torques object with the attitute control
        torques.roll = roll_ctrl_output
        torques.pithc = pitch_ctrl_output
        torques.yaw = pithch_ctrl_output
         **********************************************************************************************/
        void ComputeCustomTorques(flair::core::Euler &torques);

        /*****************************************************************************
                ComputeCustomThrust
        *******************************************************************************************
        This is a virtual function, and it is needed
        computes the altitude control
        *******************************************************************************************/
        float ComputeCustomThrust(void);

        /**********************************************************************************
        ***     ComputePositionControllers
        ***********************************************************************************
        Computes the control law for x, y and z,
        this is very important here is the backstepping 
        This sends data to:
            ComputeCustomThrust
            GetReferenceOrientation
        **********************************************************************************/
        void ComputePositionControllers(flair::core::Vector3Df &Controller);


        /*********************************************************************************
        ***             ComputeDesiredQuaternion
        **********************************************************************************
        Once the position Controller have been computed the attitude is extracted
        from the control vector.
        Once the attitude is extracted it is sent to 
            ComputeReference Orientation
        **********************************************************************************/
        void ComputeDesiredQuaternion(flair::core::Quaternion &DesiredQuaternion);


        /*********************************************************************************
        ***                 DotProduct
        **********************************************************************************
        Returns the resulto of a internal product between 3D vectors
        **********************************************************************************/
        float DotProduct(flair::core::Vector3Df &v1, flair::core::Vector3Df &v2);


        /********************************************************************************
        ****                VectorialProduct
        *********************************************************************************
        Returns the result of the vectorial product between 3D vectors
        it is used instead fo the Skew Matrix Operator
        ********************************************************************************/
        flair::core::Vector3Df VectorialProduct(flair::core::Vector3Df &m, flair::core::Vector3Df &v);


        void FleetPos(std::vector<float> &fleet_x, std::vector<float> &fleet_y);

        void SendFormationMessage(void);

        void SendHomeMessage(void);


        void SetHoldPosition(void);



        void ExtractRelativeDistances(int idDrone, std::vector<float> &reldistx, std::vector<float> &reldisty);




        void GetReferenceQuaternion(flair::core::Quaternion &refQuat, flair::core::Vector3Df &refOmega);


        void GetUAVOrientation(flair::core::Quaternion &uavQuaternion, flair::core::Vector3Df &uavAngSpeed);

        void ParseFleetPositionMessage(std::string &ParsingStr);

        void UpdatePositionLabel();








        flair::core::UdpSocket *message;

        flair::core::AhrsData *customReferenceOrientation,*customOrientation;
		flair::meta::MetaVrpnObject *uavVrpn;

        flair::gui::DoubleSpinBox *x_ref_spb, *y_ref_spb, *z_ref_spb;


        flair::gui::PushButton *HomePB;
        flair::gui::PushButton *FormationPB;


        flair::gui::Tab *myTab; 
        flair::gui::Tab *setup_myTab, *graph_law_angles, *graph_law_position;
        flair::gui::Tab *GainsTab, *AnglesGraph, *PositionGraph;

        flair::gui::Tab *MultiAgent_tab; 
        flair::gui::Tab *setup_myTab_MA, *graph_law_angles_MA, *graph_law_position_MA;


        /////////// CLASS FOR ATTITUDE CONTROL IN QUATERNION and Backstepping
        flair::filter::attQuatBkstp *attQuat;
        flair::filter::attSMC *attQuatSMC;

        //////////// POINTER TO A CLASS FOR THE BACKSTEPPING CONTROL ALONG THE X AXIS///////////
        flair::filter::xyBackstepping *xCtrl;
        flair::filter::xySMC *xCtrlSMC;


        ////////////    POINTER TO A CLASS FOR THE BACKSTEPPING CONTRO ALONG THE Y AXIS /////////////
        flair::filter::xyBackstepping *yCtrl;
        flair::filter::xySMC *yCtrlSMC;

        ////////////    POINTER TO A CLASS FOR THE BACKSTEPPING CONTROL ALONG THE Z AXIS /////////////
        flair::filter::zBackstepping *z_Ctrl;


        ////////////    POINTER TO A CLASS FOR FLIGHT FORMATION/////////////
        flair::filter::xyFleet *xFormation, *yFormation;

        ////////////    POINTER TO A CLASS FOR FLIGHT FORMATION         ////////////
        flair::filter::xyFleetPD *xFormPD, *yFormPD;



        ////////////    POINTER TO A CLASS FOR FLIGHT FORMATION         ////////////
        flair::filter::xyFleetBkstp *xFormBkstp, *yFormBkstp;

        flair::filter::xyFleetSMC   *xFormSMC, *yFormSMC;



        int IdDrone;
        int *ptrIdDrone = &IdDrone;
        std::string Name_Drone;
        std::string s;
        flair::meta::MetaVrpnObject *ArrayDrones[3];

        flair::gui::DoubleSpinBox *F1x[3][3], *F2x[3][3];
        flair::gui::DoubleSpinBox *F1y[3][3], *F2y[3][3];

        flair::gui::DoubleSpinBox *x_Fleet, *y_Fleet;
        flair::gui::DoubleSpinBox *xGainLead, *yGainLead;

        flair::gui::PushButton *f1PB;
        flair::gui::PushButton *f2PB;
        flair::gui::PushButton *LandPB1, *LandPB2;

        flair::gui::PushButton *HomePB_01;
        flair::gui::PushButton *FormationPB_01;
        flair::gui::Label *HomePosX;
        flair::gui::Label *HomePosY;
        flair::gui::Label *HomeXText;
        flair::gui::Label *HomeYText;


        flair::gui::PushButton *btnUpdateFleetPos;
        
        float LeaderPosX;
        float LeaderPosY;


        int Form_index = 0;


        flair::core::Vector3Df Pos2Hold{0,0,0};

};

#endif // MultiAgentApp_H
