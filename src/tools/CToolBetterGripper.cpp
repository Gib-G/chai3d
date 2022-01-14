#include "CToolBetterGripper.h"
#include "graphics/CTriangleArray.h"

namespace chai3d {

    cToolBetterGripper::cToolBetterGripper(cWorld* a_parentWorld) :cGenericTool(a_parentWorld)
    {
        // default gripper workspace
        m_gripperWorkspaceScale = 1.0;

        // create a haptic point contact for the thumb
        m_hapticPointThumb = new cHapticPoint(this);
        m_hapticPoints.push_back(m_hapticPointThumb);

        // create a haptic point contact for the finger
        m_hapticPointFinger = new cHapticPoint(this);
        m_hapticPoints.push_back(m_hapticPointFinger);

        // show proxy spheres only
        setShowContactPoints(true, false);
    }


    //==============================================================================
    /*!
        Destructor of cToolBetterGripper.
    */
    //==============================================================================
    cToolBetterGripper::~cToolBetterGripper()
    {
        delete  m_hapticPointThumb;
        delete  m_hapticPointFinger;
    }


    //==============================================================================
    /*!
        This method computes the interaction forces between the tool and all
        objects located inside the virtual world.
    */
    //==============================================================================
    void cToolBetterGripper::computeInteractionForces()
    {
        // convert the angle of the gripper into a position in device coordinates. 
        // this value is device dependent.
        double gripperPositionFinger = 0.0;
        double gripperPositionThumb = 0.0;

        if (m_hapticDevice->m_specifications.m_model == C_HAPTIC_DEVICE_OMEGA_7)
        {
            gripperPositionFinger = 0.040 * cSinRad(m_gripperAngle + cDegToRad(1.0));
            gripperPositionThumb = 0.040 * cSinRad(-m_gripperAngle + cDegToRad(-1.0));
        }
        else if (m_hapticDevice->m_specifications.m_model == C_HAPTIC_DEVICE_SIGMA_7)
        {
            gripperPositionFinger = 0.040 * cSinRad(m_gripperAngle + cDegToRad(1.0));
            gripperPositionThumb = 0.040 * cSinRad(-m_gripperAngle + cDegToRad(-1.0));
        }
        else if (m_hapticDevice->m_specifications.m_model == C_HAPTIC_DEVICE_XTH_2)
        {
            gripperPositionFinger = 0.040 * cSinRad(m_gripperAngle + cDegToRad(1.0));
            gripperPositionThumb = 0.040 * cSinRad(-m_gripperAngle + cDegToRad(-1.0));
        }
        else
        {
            gripperPositionFinger = 0.040 * cSinRad(m_gripperAngle + cDegToRad(1.0));
            gripperPositionThumb = 0.040 * cSinRad(-m_gripperAngle + cDegToRad(-1.0));
        }

        // compute new position of thumb and finger 
        cVector3d lineFingerThumb = getGlobalRot().getCol1();
        cVector3d pFinger = m_gripperWorkspaceScale * m_workspaceScaleFactor * gripperPositionFinger * lineFingerThumb;
        cVector3d pThumb = m_gripperWorkspaceScale * m_workspaceScaleFactor * gripperPositionThumb * lineFingerThumb;

        cVector3d posFinger, posThumb;
        if (m_hapticDevice->m_specifications.m_rightHand)
        {
            posFinger = m_deviceGlobalPos + cMul(m_deviceGlobalRot, (1.0 * pFinger));
            posThumb = m_deviceGlobalPos + cMul(m_deviceGlobalRot, (1.0 * pThumb));
        }
        else
        {
            posFinger = m_deviceGlobalPos + cMul(m_deviceGlobalRot, (-1.0 * pFinger));
            posThumb = m_deviceGlobalPos + cMul(m_deviceGlobalRot, (-1.0 * pThumb));
        }

        // compute forces
        cVector3d forceThumb = m_hapticPointThumb->computeInteractionForces(posThumb,
            m_deviceGlobalRot,
            m_deviceGlobalLinVel,
            m_deviceGlobalAngVel);

        cVector3d forceFinger = m_hapticPointFinger->computeInteractionForces(posFinger,
            m_deviceGlobalRot,
            m_deviceGlobalLinVel,
            m_deviceGlobalAngVel);

        // compute torques
        double scl = 0.0;
        double factor = m_gripperWorkspaceScale * m_workspaceScaleFactor;
        if (factor > 0.0)
        {
            scl = 1.0 / factor;
        }
        cVector3d torque = scl * cAdd(cCross(cSub(posThumb, m_deviceGlobalPos), forceThumb), cCross(cSub(posFinger, m_deviceGlobalPos), forceFinger));

        // compute gripper force
        double gripperForce = 0.0;

        if ((m_hapticDevice->m_specifications.m_model == C_HAPTIC_DEVICE_OMEGA_7) ||
            (m_hapticDevice->m_specifications.m_model == C_HAPTIC_DEVICE_SIGMA_7))
        {
            cVector3d dir = posFinger - posThumb;
            if (dir.length() > 0.00001)
            {
                dir.normalize();
                cVector3d force = cProject(forceFinger, dir);
                gripperForce = force.length();
                if (force.length() > 0.001)
                {
                    double angle = cAngle(dir, force);
                    if ((angle > C_PI / 2.0) || (angle < -C_PI / 2.0)) gripperForce = -gripperForce;
                }
            }
        }

        // gripper damping
        double gripperAngularVelocity = 0.0;
        m_hapticDevice->getGripperAngularVelocity(gripperAngularVelocity);
        double gripperDamping = -0.1 * m_hapticDevice->m_specifications.m_maxGripperAngularDamping * gripperAngularVelocity;

        // finalize forces, torques and gripper force
        cVector3d globalForce = forceThumb + forceFinger;
        cVector3d globalTorque = torque;
        gripperForce = gripperForce + gripperDamping;

        // update computed forces to tool
        setDeviceGlobalForce(globalForce);
        setDeviceGlobalTorque(globalTorque);
        setGripperForce(gripperForce);
    }


    //==============================================================================
    /*!
        This method renders the current tool using OpenGL.
        \param  a_options  Rendering options.
    */
    //==============================================================================
    void cToolBetterGripper::render(cRenderOptions& a_options)
    {
        ///////////////////////////////////////////////////////////////////////
        // render haptic points
        ///////////////////////////////////////////////////////////////////////
        int numContactPoint = (int)(m_hapticPoints.size());
        for (int i = 0; i < numContactPoint; i++)
        {
            // get next haptic point
            cHapticPoint* nextContactPoint = m_hapticPoints[i];

            // render tool
            nextContactPoint->render(a_options);
        }

        ///////////////////////////////////////////////////////////////////////
        // render mesh image
        ///////////////////////////////////////////////////////////////////////
        if (m_image != NULL)
        {
            m_image->renderSceneGraph(a_options);
        }
    }

} 