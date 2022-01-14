#ifndef CToolBetterGripperH
#define CToolBetterGripperH
//------------------------------------------------------------------------------
#include "tools/CGenericTool.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
    //------------------------------------------------------------------------------

    //==============================================================================
    /*!
        \file       CToolBetterGripper.h
        \brief
        Implements a gripper using two haptic points.
    */
    //==============================================================================

    //==============================================================================
    /*!
        \class      cToolBetterGripper
        \ingroup    tools
        \brief
        This class implements a gripper using a two haptic points.
        \details
        cToolBetterGripper implements a gripper tool using two haptic points.
        The distance between both haptic points is controlled by the haptic gripper.
        The resulting forces computed at each haptic point are converted into
        a force, torque, and gripper force, which are then sent to the haptic device.
    */
    //==============================================================================
    class cToolBetterGripper : public cGenericTool
    {
        //--------------------------------------------------------------------------
        // CONSTRUCTOR & DESTRUCTOR:
        //--------------------------------------------------------------------------

    public:

        //! Constructor of cToolBetterGripper.
        cToolBetterGripper(cWorld* a_parentWorld);

        //! Destructor of cToolBetterGripper.
        virtual ~cToolBetterGripper();


        //--------------------------------------------------------------------------
        // PUBLIC MEMBERS
        //--------------------------------------------------------------------------

    public:

        // Haptic point modeling the thumb.
        cHapticPoint* m_hapticPointThumb;

        // Haptic point modeling the index.
        cHapticPoint* m_hapticPointFinger;


        //--------------------------------------------------------------------------
        // PUBLIC METHODS
        //--------------------------------------------------------------------------

    public:

        //! This method computes the interaction forces between the gripper tool and environment.
        virtual void computeInteractionForces();

        //! This method renders the tools using OpenGL.
        virtual void render(cRenderOptions& a_options);

        //! This method sets a workspace scale factor for the gripper.
        virtual void setGripperWorkspaceScale(double a_gripperWorkspaceScale) { m_gripperWorkspaceScale = fabs(a_gripperWorkspaceScale); }

        //! This method returns the workspace scale factor of the gripper.
        virtual double getGripperWorskpaceScale() { return (m_gripperWorkspaceScale); }


        //--------------------------------------------------------------------------
        // MEMBERS
        //--------------------------------------------------------------------------

    protected:

        // Workspace scale factor of force gripper.
        double m_gripperWorkspaceScale;
    };

    //------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------