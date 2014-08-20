#ifndef STIM300_TYPE_HPP
#define STIM300_TYPE_HPP

#include <base/time.h>
#include <base/temperature.h>
#include <vector>

namespace imu_stim300 {

    /** Sensornor revision names **/
    enum FIRMWARE_REVISIONS{REV_A, REV_B, REV_C, REV_D};

    /** Magnetic declination mode **/
    enum MAGNETIC_DECLINATION_MODE{EAST, WEST};

    /** STIM300 temperature sensor **/
    struct Temperature
    {
	base::Time time;
	std::vector<base::Temperature> temp;
	
	void resize(int size)
        {
                temp.resize(size);
        }

        size_t size()
        {
            return temp.size();
        }
    };

    /** Filter Configuration **/
    struct FilterConfiguration
    {
        std::string source_frame_name; //Output Frame name. Transformation: source -> target

        std::string target_frame_name; //Output Frame name. Transformation: source -> target

        bool use_samples_as_theoretical_gravity;//Inclinometers are more stable than accelerometers at initial time.
                                                    //They cloud be use as theoretical local gravity value instead of using
                                                    //some models as WGS-84 ellipsoid Earth.
                                                    //It will use inclinometers in case use_inclinometers is true
                                                    //and accelerometers otherwise.
        bool use_inclinometers;//Some IMU provide inclinometers as fast and more accurate solution for initial leveling.
                                //Set True or False to use inclinometers values or not.
                                //Note: Check if the IMU has inclinometers information.

        bool use_magnetometers;// Some IMUS provides Magnetic information.
                                // Set to true or false in case you want to correct heading with magnetometers

        unsigned int initial_alignment_samples;// Number of samples to compute the initial alignment of the imu frame to the local geographic coordinate frame.
                                            // This step involves the gravity vector (leveling) and finding the true North (gyrocompassing).
                                            //Set to zero in case zero attitude is desired as initial orientation from an arbitrary frame.

        double correction_frequency; //frequency of the correction step.Set to zero or the same sampling frequency to correct at the same time than predict

    };

    //Data type for the Accelerometers measurement characteristic
    struct InertialNoiseParameters
    {
        /********************************/
        /** Inertial Sensor Properties **/
        /********************************/

        double bandwidth; //Inertial sensors bandwidth in Hertz.
        //This is characteristic of the sensor and should be equal
        //or smaller than the sampling rate but normaly it should respect Nyquist frequency

        /** Accelerometers Noise **/
        base::Vector3d biasoffset;//bias offset in static regimen for the Accelerometers
        base::Vector3d randomwalk;//velocity/ random walk for accelerometers (m/s/sqrt(s))
        base::Vector3d raterandomwalk;//acceleration random walk for accelerometers (m/s^2/sqrt(s))
        base::Vector3d biasinstability;//accelerometers bias instability (m/s^2)
        base::Vector3d resolution;//minimum accelerometers resolution (m/s^2)
    };

    //Data type to know the location
    struct LocationConfiguration
    {
        double latitude;//Latitude in radians
        double longitude;//Longitude in radians
        double altitude;//Altitude in meters
        double magnetic_declination;//Declination in radians
        imu_stim300::MAGNETIC_DECLINATION_MODE magnetic_declination_mode;//The declination is positive when the magnetic north is East of true north
                                                                    //EAST means positive declination and WEST means negative declination.
        double dip_angle;//Dip angle in radians
    };

    /** Adaptive Measurement Configuration. Variables for the attitude estimation inside the algorithm **/
    struct AdaptiveAttitudeConfig
    {
        unsigned int M1; /** Parameter for adaptive algorithm (to estimate Uk which is not directly observable) */
        unsigned int M2; /** Parameter for adaptive algorithm (to prevent false entering in no-external acc mode) */
        double gamma; /** Parameter for adaptive algorithm. Only entering when Qstart (adaptive cov. matrix) is greater than RHR'+Ra */

        void reset()
        {
            M1 = 0;
            M2 = 0;
            gamma = 0.0;
            return;
        }

    };

}

#endif
