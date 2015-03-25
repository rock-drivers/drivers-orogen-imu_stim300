/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef STIM300_TASK_TASK_HPP
#define STIM300_TASK_TASK_HPP

#include "imu_stim300/TaskBase.hpp"

/** Driver library **/
#include <imu_stim300/Stim300Base.hpp>
#include <imu_stim300/Stim300RevB.hpp>
#include <imu_stim300/Stim300RevD.hpp>

#include <quater_ikf/Ikf.hpp> /** IKF filter*/

#include <aggregator/TimestampEstimator.hpp>
#include <rtt/extras/FileDescriptorActivity.hpp>

namespace imu_stim300 {

    /** WGS-84 ellipsoid constants (Nominal Gravity Model and Earth angular velocity) **/
    static const int Re = 6378137; /** Equatorial radius in meters **/
    static const int Rp = 6378137; /** Polar radius in meters **/
    static const double ECC = 0.0818191908426; /** First eccentricity **/
    static const double GRAVITY = 9.79766542; /** Mean value of gravity value in m/s^2 **/
    static const double GWGS0 = 9.7803267714; /** Gravity value at the equator in m/s^2 **/
    static const double GWGS1 = 0.00193185138639; /** Gravity formula constant **/
    static const double EARTHW = 7.292115e-05; /** Earth angular velocity in rad/s **/

     enum CONST {
            IKFSTATEVECTORSIZE = filter::Ikf<double, true, true>::IKFSTATEVECTORSIZE
    };

    static const double GRAVITY_MARGIN = 0.3; /** Accepted error for the gravity value in [m/s^2] **/

    /*! \class Task 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * 
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','imu_stim300::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class Task : public TaskBase
    {
	friend class TaskBase;

    protected:

        /******************************/
        /*** Control Flow Variables ***/
        /******************************/

        /** Initial Attitude **/
	    bool initAttitude;

        /** Index for initializing attitude **/
        unsigned int initial_alignment_idx;

        /**************************/
        /*** Property Variables ***/
        /**************************/

        /** Filter configuration values **/
        FilterConfiguration config;

        /** Accelerometers noise parameters **/
        InertialNoiseParameters accnoise;

        /** Gyroscopes noise parameters **/
        InertialNoiseParameters gyronoise;

        /** Inclinometers noise parameters **/
        InertialNoiseParameters incnoise;

        /** Adaptive Measurement Configuration **/
        AdaptiveAttitudeConfig adaptiveconfigAcc;
        AdaptiveAttitudeConfig adaptiveconfigInc;

        /** Location configuration variables **/
        LocationConfiguration location;

        /**************************/
        /*** Internal Variables ***/
        /**************************/

        base::Time prev_ts;

        int correction_numbers, correction_idx;

        double sampling_frequency;

        /** Driver variables **/
        imu_stim300::Stim300Base *imu_stim300_driver;
        aggregator::TimestampEstimator* timestamp_estimator;

        /** Correction  Variables**/
        Eigen::Vector3d correctionAcc, correctionInc;

        /** Initial values of Accelerometers/Inclinometers for Pitch and Roll calculation */
        Eigen::Matrix <double, 3, Eigen::Dynamic> initial_alignment_gyro;
        Eigen::Matrix <double, 3, Eigen::Dynamic> initial_alignment_acc;

        filter::Ikf<double, true, true> myfilter; /** The adaptive Indirect Kalman filter */

        Eigen::Quaterniond deltaquat, deltahead, attitude;

        Eigen::Matrix4d oldomega;

        /***************************/
        /** Output port variables **/
        /***************************/

        base::samples::RigidBodyState orientation_out; /** the output orientation **/

    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
         Task(std::string const& name = "imu_stim300::Task");
	
        /** TaskContext constructor for Task 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        Task(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Task
         */
        ~Task();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();


        /** Performs heading independent integration
        */
        Eigen::Quaternion<double> deltaHeading(const Eigen::Vector3d &angvelo, Eigen::Matrix4d &oldomega, const double delta_t);


        /** @brief Port out the values
	    */
        void outputPortSamples(imu_stim300::Stim300Base *driver, filter::Ikf<double, true, true> &myfilter, base::samples::IMUSensors &imusamples);

        /**
        * @brief This computes the theoretical gravity value according to the WGS-84 ellipsoid Earth model.
        *
        * @author Javier Hidalgo Carrio.
        *
        * @param[in] latitude double the latitude value in radian
        * @param[in] altitude double with the altitude value in meters
        *
        * @return double. the theoretical value of the local gravity
        *
        */
        static double GravityModel(double latitude, double altitude)
        {
            double g; /** g magnitude at zero altitude **/

            /** Nominal Gravity model **/
            g = GWGS0*((1+GWGS1*pow(sin(latitude),2))/sqrt(1-pow(ECC,2)*pow(sin(latitude),2)));

            /** Gravity affects by the altitude (aprox the value r = Re **/
            g = g*pow(Re/(Re+altitude), 2);

                #ifdef DEBUG_PRINTS
            std::cout<<"[STIM300_CLASS] Theoretical gravity for this location (WGS-84 ellipsoid model): "<< g<<" [m/s^2]\n";
                #endif

            return g;
        };

        /**
        * @brief Subtract the Earth rotation from the gyroscopes readout
        *
        * This function computes the subtraction of the rotation of the Earth (EARTHW)
        * from the gyroscope values. This function uses quaternion of transformation from
        * the geographic to body frame and the latitude in radians.
        *
        * @author Javier Hidalgo Carrio.
        *
        * @param[in, out] u angular velocity in body frame
        * @param[in] q quaternion from body to geographic(world) frame v_body = q_body_2_geo * v_geo
        * @param[in] latitude location latitude angle in radians
        *
        * @return void
        *
        */
        static void SubtractEarthRotation(Eigen::Vector3d &u, const Eigen::Quaterniond &q, const double latitude)
        {
            Eigen::Vector3d v (EARTHW*cos(latitude), 0, EARTHW*sin(latitude)); /** vector of earth rotation components expressed in the geographic frame according to the latitude **/

            /** Compute the v vector expressed in the body frame **/
            v = q * v;

            #ifdef DEBUG_PRINTS
            std::cout<<"[STIM300_CLASS] Earth Rotation:"<<v<<"\n";
            #endif

            /** Subtract the earth rotation to the vector of inputs (u = u-v**/
            u  = u - v;

            return;
        };

        /**
        * @brief Delta quaternion rotation. Integration of small (given by the current angular velo) variation in attitude.
        */
        static Eigen::Quaternion<double> deltaQuaternion(const Eigen::Vector3d &angvelo, const Eigen::Matrix4d &oldomega4, const Eigen::Matrix4d &omega4, const double dt)
        {
            Eigen::Vector4d quat;
            quat<< 1.00, 0.00, 0.00, 0.00; /**Identity quaternion */

            /** Third-order gyroscopes integration accuracy **/
            quat = (Eigen::Matrix<double,4,4>::Identity() +(0.75 * omega4 *dt)-(0.25 * oldomega4 * dt) -
            ((1.0/6.0) * angvelo.squaredNorm() * pow(dt,2) *  Eigen::Matrix<double, 4, 4>::Identity()) -
            ((1.0/24.0) * omega4 * oldomega4 * pow(dt,2)) - ((1.0/48.0) * angvelo.squaredNorm() * omega4 * pow(dt,3))) * quat;

            Eigen::Quaternion<double> deltaq;
            deltaq.w() = quat(0);
            deltaq.x() = quat(1);
            deltaq.y() = quat(2);
            deltaq.z() = quat(3);
            deltaq.normalize();

            return deltaq;
        };

    public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    };
}

#endif

