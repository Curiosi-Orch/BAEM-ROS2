#ifndef KUKASUNRISE_H
#define KUKASUNRISE_H

#include "datagramserver.h"
#include <Erl.h>

class KukaSunrise
{

private:

    constexpr static size_t RCV_PACKETSIZE = 400;
    constexpr static size_t SND_PACKETSIZE = 300;
    constexpr static uint8_t BYTE_HIGH =  (uint8_t) 255;
    constexpr static uint8_t BYTE_LOW  =  (uint8_t)   0;

    constexpr static size_t Lrcv_MSR_TRANSFORM  =                            0;
    constexpr static size_t Lrcv_CMD_TRANSFORM  =  Lrcv_MSR_TRANSFORM + 12 * 8;
    constexpr static size_t Lrcv_JOINTS         =  Lrcv_CMD_TRANSFORM + 12 * 8;
    constexpr static size_t Lrcv_STIFFNESS      =  Lrcv_JOINTS        +  7 * 8;
    constexpr static size_t Lrcv_DAMPING        =  Lrcv_STIFFNESS     +  7 * 8;
    constexpr static size_t Lrcv_TIMESTAMP      =  Lrcv_DAMPING       +  7 * 8;
    constexpr static size_t Lrcv_F_IMPEDANCE    =  Lrcv_TIMESTAMP     +  1 * 8;
    constexpr static size_t Lrcv_F_EXIT_1       =  RCV_PACKETSIZE - 2;
    constexpr static size_t Lrcv_F_EXIT_2       =  RCV_PACKETSIZE - 1;

    constexpr static size_t Lsnd_TRANSFORM      =                            0;
    constexpr static size_t Lsnd_JOINTS         =  Lsnd_TRANSFORM     + 12 * 8;
    constexpr static size_t Lsnd_STIFFNESS      =  Lsnd_JOINTS        +  7 * 8;
    constexpr static size_t Lsnd_DAMPING        =  Lsnd_STIFFNESS     +  7 * 8;
    constexpr static size_t Lsnd_TIMESTAMP      =  Lsnd_DAMPING       +  7 * 8;
    constexpr static size_t Lsnd_F_ENABLE       =  Lsnd_TIMESTAMP     +  1 * 8;
    constexpr static size_t Lsnd_F_JOINTS       =  Lsnd_F_ENABLE      +      1;
    constexpr static size_t Lsnd_F_IMPEDANCE    =  Lsnd_F_JOINTS      +      1;
    constexpr static size_t Lsnd_F_EXIT_1       =  SND_PACKETSIZE - 2;
    constexpr static size_t Lsnd_F_EXIT_2       =  SND_PACKETSIZE - 1;

public:

    KukaSunrise();
    ~KukaSunrise();

    bool connect(const std::string& hostname, const int& localPort, const int& iiwaPort,
                 const int& inital_timeout, const int& comm_timeout);
    bool close();
    bool isConnected() const;

    void moveToDefaultPosition();
    void moveToPose(const Erl::Transformd& Pose, const double& time_ms);

    inline Erl::Transformd getMsrTransform() const
    {
        std::lock_guard<std::mutex> lock(rcvLock_);
        return rcv_msr_transform_;
    }

    inline Erl::Transformd getDestTransform() const
    {
        std::lock_guard<std::mutex> lock(rcvLock_);
        return rcv_dest_transform_;
    }

    inline Erl::Transformd getSetDestTransform() const
    {
        std::lock_guard<std::mutex> lock(sndLock_);
        return snd_transform_;
    }

    inline Eigen::Matrix<double, 7, 1> getJoints() const
    {
        std::lock_guard<std::mutex> lock(rcvLock_);
        return rcv_joints_;
    }

    inline Eigen::Matrix<double, 7, 1> getSetJoints() const
    {
        std::lock_guard<std::mutex> lock(sndLock_);
        return snd_joints_;
    }

    inline Eigen::Matrix<double, 7, 1> getStiffness() const
    {
        std::lock_guard<std::mutex> lock(rcvLock_);
        return rcv_stiffness_;
    }

    inline Eigen::Matrix<double, 7, 1> getDamping() const
    {
        std::lock_guard<std::mutex> lock(rcvLock_);
        return rcv_damping_;
    }

    inline bool isUsingImpedanceMode() const
    {
        std::lock_guard<std::mutex> lock(rcvLock_);
        return ( rcv_flag_impedance_ == BYTE_HIGH );
    }

    inline void setTransform(const Erl::Transformd& destination)
    {
        std::lock_guard<std::mutex> lock(sndLock_);
        snd_transform_ = destination;
        snd_flag_joints_ = BYTE_LOW;
    }

    inline void setJoints(const Eigen::Matrix<double, 7, 1>& joints)
    {
        std::lock_guard<std::mutex> lock(sndLock_);
        snd_joints_ = joints;
        snd_flag_joints_ = BYTE_HIGH;
    }

    inline void setStiffness(const Eigen::Matrix<double, 7, 1>& stiffness)
    {
        std::lock_guard<std::mutex> lock(sndLock_);
        snd_stiffness_ = stiffness;
    }

    inline void setStiffness(const Eigen::Matrix<double, 6, 1>& stiffness)
    {
        std::lock_guard<std::mutex> lock(sndLock_);
        snd_stiffness_.head<6>() = stiffness;
    }

    inline void setDamping(const Eigen::Matrix<double, 7, 1>& damping)
    {
        std::lock_guard<std::mutex> lock(sndLock_);
        snd_damping_ = damping;
    }

    inline void setDamping(const Eigen::Matrix<double, 6, 1>& damping)
    {
        std::lock_guard<std::mutex> lock(sndLock_);
        snd_damping_.head<6>() = damping;
    }

    inline void setEnableOutput(const bool& enable)
    {
        std::lock_guard<std::mutex> lock(sndLock_);
        if (enable)
            snd_flags_enable_ = BYTE_HIGH;
        else
            snd_flags_enable_ = BYTE_LOW;
    }

    inline void applyGravityCompensationSettings()
    {
        setLoopBackPose(true);
        setStiffness(Erl::Vector6d(0,0,0,0,0,0));
        setDamping(Erl::Vector6d(0.1, 0.1, 0.1, 0.1, 0.1, 0.1));
    }

    inline void disableGravityCompensationSettings(Erl::Vector6d stiffness = Erl::Vector6d(2000,2000,2000,100,100,100),
                                                   Erl::Vector6d damping   = Erl::Vector6d(0.7, 0.7, 0.7, 0.7, 0.7, 0.7))
    {
        setStiffness(stiffness);
        setDamping(damping);
        setLoopBackPose(false);
    }

    inline void setLoopBackPose(const bool& enable)
    {
        std::lock_guard<std::mutex> lock1(sndLock_);
        std::lock_guard<std::mutex> lock2(rcvLock_);
        snd_transform_ = rcv_msr_transform_;
        loopBackMsrTransform_ = enable;
    }

    inline bool isOutputEnabled() const
    {
        std::lock_guard<std::mutex> lock(sndLock_);
        return ( snd_flags_enable_ == BYTE_HIGH );
    }

    inline bool isPoseLoopBackEnabled() const
    {
        std::lock_guard<std::mutex> lock(sndLock_);
        return loopBackMsrTransform_;
    }

    std::thread * getThread() { return &commThread_; }

private:

    KukaSunrise(const KukaSunrise& K)  = delete;
    KukaSunrise& operator = (const KukaSunrise& K ) = delete;

    void communicationLoop(const int &comm_timeout);
    void initialiseCommData();
    void performCloseSequence();

    void updateCommandData();
    void updateMeasuredData();

    DatagramServer connection_;
    std::thread commThread_;
    std::atomic<bool> commRunning_;
    std::atomic<bool> closedOK_;

    int64_t snd_timestamp_;
    int64_t rcv_timestamp_;
    uint8_t rcvData_ [RCV_PACKETSIZE];
    uint8_t sndData_ [SND_PACKETSIZE];

    /// When locking both, always lock sndLock_ first.
    mutable std::mutex sndLock_;
    mutable std::mutex rcvLock_;

    /// Measured data
    Erl::Transformd rcv_msr_transform_;
    Erl::Transformd rcv_dest_transform_;
    Eigen::Matrix<double, 7, 1> rcv_joints_;
    Eigen::Matrix<double, 7, 1> rcv_stiffness_;
    Eigen::Matrix<double, 7, 1> rcv_damping_;
    uint8_t rcv_flag_impedance_;

    /// Command data
    Erl::Transformd snd_transform_;
    Eigen::Matrix<double, 7, 1> snd_joints_;
    Eigen::Matrix<double, 7, 1> snd_stiffness_;
    Eigen::Matrix<double, 7, 1> snd_damping_;
    uint8_t snd_flags_enable_;
    uint8_t snd_flag_joints_;
    uint8_t snd_flag_impedance_;
    bool    loopBackMsrTransform_;

};

#endif // KUKASUNRISE_H
