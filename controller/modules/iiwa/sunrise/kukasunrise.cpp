#include "kukasunrise.h"

#if defined(__GNUC__) || defined(__MINGW32__) || defined(__MINGW64__)

#define byteswap16 __builtin_bswap16
#define byteswap32 __builtin_bswap32
#define byteswap64 __builtin_bswap64

#elif _MSC_VER

#define byteswap16 _byteswap_ushort
#define byteswap32 _byteswap_ulong
#define byteswap64 _byteswap_uint64

#endif

#ifndef PLATFORM_BIG_ENDIAN
#define PLATFORM_LITTLE_ENDIAN
#endif

namespace details
{

template <typename T, size_t n>
struct ByteswapImpl;

template <typename T>
struct ByteswapImpl<T, 2> {
  T operator()(const T& arg) const
  {
      T val;
      *((uint16_t*)&val) = byteswap16 ( *(uint16_t*)&arg );
      return val;
  }
};

template <typename T>
struct ByteswapImpl<T, 4> {
  T operator()(const T& arg) const
  {
      T val;
      *((uint32_t*)&val) = byteswap32 ( *(uint32_t*)&arg );
      return val;
  }
};

template <typename T>
struct ByteswapImpl<T, 8> {
  T operator()(const T& arg) const
  {
      T val;
      *((uint64_t*)&val) = byteswap64 ( *(uint64_t*)&arg );
      return val;
  }
};

template <typename T>
inline T byteswap(const T& arg) { return ByteswapImpl<T, sizeof(T)>()(arg); }

template <typename T>
inline void copyToBytes_swp(T *src, uint8_t *bytes, const unsigned& l_bytes)
{
    for (unsigned i = 0; i < l_bytes/sizeof(T); i++)
    {
        T tmp = byteswap( src[i] );
        ::memcpy( &((T*)bytes)[i], &tmp, sizeof(T) );
    }
}

template <typename T>
inline void copyFromBytes_swp(uint8_t *bytes, T *dest, const unsigned& l_bytes)
{
    for (unsigned i = 0; i < l_bytes/sizeof(T); i++)
    {
        dest[i] = byteswap( ((T*)bytes)[i] );
    }
}

}

template <typename T>
inline void copyToBytes(T *src, uint8_t *bytes, const unsigned& l_bytes)
{
#ifdef PLATFORM_LITTLE_ENDIAN
    details::copyToBytes_swp(src, bytes, l_bytes);
#else
    ::memcpy(bytes, src, l_bytes);
#endif
}

template <typename T>
inline void copyFromBytes(uint8_t *bytes, T *dest, const unsigned& l_bytes)
{
#ifdef PLATFORM_LITTLE_ENDIAN
    details::copyFromBytes_swp(bytes, dest, l_bytes);
#else
    ::memcpy(dest, bytes, l_bytes);
#endif
}

constexpr size_t KukaSunrise::RCV_PACKETSIZE;
constexpr size_t KukaSunrise::SND_PACKETSIZE ;
constexpr uint8_t KukaSunrise::BYTE_HIGH;
constexpr uint8_t KukaSunrise::BYTE_LOW;

constexpr size_t KukaSunrise::Lrcv_MSR_TRANSFORM;
constexpr size_t KukaSunrise::Lrcv_CMD_TRANSFORM;
constexpr size_t KukaSunrise::Lrcv_JOINTS;
constexpr size_t KukaSunrise::Lrcv_STIFFNESS;
constexpr size_t KukaSunrise::Lrcv_DAMPING;
constexpr size_t KukaSunrise::Lrcv_TIMESTAMP;
constexpr size_t KukaSunrise::Lrcv_F_IMPEDANCE;
constexpr size_t KukaSunrise::Lrcv_F_EXIT_1;
constexpr size_t KukaSunrise::Lrcv_F_EXIT_2;

constexpr size_t KukaSunrise::Lsnd_TRANSFORM;
constexpr size_t KukaSunrise::Lsnd_JOINTS;
constexpr size_t KukaSunrise::Lsnd_STIFFNESS;
constexpr size_t KukaSunrise::Lsnd_DAMPING;
constexpr size_t KukaSunrise::Lsnd_TIMESTAMP;
constexpr size_t KukaSunrise::Lsnd_F_ENABLE;
constexpr size_t KukaSunrise::Lsnd_F_JOINTS;
constexpr size_t KukaSunrise::Lsnd_F_IMPEDANCE;
constexpr size_t KukaSunrise::Lsnd_F_EXIT_1;
constexpr size_t KukaSunrise::Lsnd_F_EXIT_2;

KukaSunrise::KukaSunrise()
    : snd_timestamp_(0)
    , rcv_timestamp_(-1)
    , loopBackMsrTransform_(false)
{
    commRunning_.store(false);
}

KukaSunrise::~KukaSunrise()
{
    if (isConnected())
        close();
}

bool KukaSunrise::connect(const std::string& hostname, const int& localPort, const int& iiwaPort,
                          const int& inital_timeout, const int& comm_timeout)
{
    if ( !connection_.open(hostname, localPort, iiwaPort) ) {
        return false;
    }

    ::memset(rcvData_, (int)BYTE_LOW, RCV_PACKETSIZE);
    ::memset(sndData_, (int)BYTE_LOW, SND_PACKETSIZE);

    bool received = connection_.receive(rcvData_, RCV_PACKETSIZE, inital_timeout);
    if ( !received )
    {
        connection_.close();
        return false;
    }

    initialiseCommData();
    commRunning_.store(true);
    commThread_ = std::thread(&KukaSunrise::communicationLoop, this, comm_timeout);

    /// Sleep to give some breathing room for the kuka side after starting communication.
    /// If commands are spammed right away, the kuka may initially be jerky.
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    return true;
}

void KukaSunrise::initialiseCommData()
{
    /// Initialise command data buffer for redundancy
    ::memcpy(&sndData_[Lsnd_TRANSFORM]   , &rcvData_[Lrcv_CMD_TRANSFORM] , 12 * 8);
    ::memcpy(&sndData_[Lsnd_JOINTS]      , &rcvData_[Lrcv_JOINTS]        ,  7 * 8);
    ::memcpy(&sndData_[Lsnd_STIFFNESS]   , &rcvData_[Lrcv_STIFFNESS]     ,  7 * 8);
    ::memcpy(&sndData_[Lsnd_DAMPING]     , &rcvData_[Lrcv_DAMPING]       ,  7 * 8);
    ::memcpy(&sndData_[Lsnd_F_IMPEDANCE] , &rcvData_[Lrcv_F_IMPEDANCE]   ,      1);

    sndData_[Lsnd_F_ENABLE] = BYTE_HIGH;
    sndData_[Lsnd_F_JOINTS] = BYTE_LOW;

    /// Initialise command data variables
    double Tr[12];
    copyFromBytes( &rcvData_[Lrcv_CMD_TRANSFORM], Tr, 12 * 8 );
    snd_transform_ = Erl::Transformd::fromPointer(Tr, Erl::RowMajor);

    copyFromBytes( &rcvData_[Lrcv_JOINTS],    snd_joints_.data(),    7*8 );
    copyFromBytes( &rcvData_[Lrcv_STIFFNESS], snd_stiffness_.data(), 7*8 );
    copyFromBytes( &rcvData_[Lrcv_DAMPING],   snd_damping_.data(),   7*8 );

    snd_timestamp_ = 0;
    snd_flags_enable_   = BYTE_HIGH;
    snd_flag_joints_    = BYTE_LOW;
    snd_flag_impedance_ = rcvData_[Lrcv_F_IMPEDANCE];


    updateMeasuredData();
}

void KukaSunrise::communicationLoop(const int& comm_timeout)
{
    bool commOK = true;
    
    while( commRunning_.load() == true && commOK)
    {
        commOK = connection_.receive(rcvData_, RCV_PACKETSIZE, comm_timeout);
        updateMeasuredData();

        updateCommandData();
        copyToBytes(&snd_timestamp_, &sndData_[Lsnd_TIMESTAMP], sizeof(snd_timestamp_));
        snd_timestamp_++;
        connection_.send(sndData_, SND_PACKETSIZE);
    }

    bool done = false;
    bool closeComOK = true;
    sndData_[Lsnd_F_EXIT_1] = BYTE_HIGH;
    sndData_[Lsnd_F_EXIT_2] = BYTE_HIGH;

    while (done == false && closeComOK == true)
    {
        copyToBytes(&snd_timestamp_, &sndData_[Lsnd_TIMESTAMP], sizeof(snd_timestamp_));
        snd_timestamp_++;
        connection_.send(sndData_, SND_PACKETSIZE);

        closeComOK = connection_.receive(rcvData_, RCV_PACKETSIZE, comm_timeout);

        if (rcvData_[Lrcv_F_EXIT_1] == BYTE_HIGH && rcvData_[Lrcv_F_EXIT_2] == BYTE_HIGH)
            done = true;
    }

    closedOK_.store(closeComOK);
}

void KukaSunrise::updateCommandData()
{
    std::lock_guard<std::mutex> lock(sndLock_);

    if (loopBackMsrTransform_)
    {
        ::memcpy(&sndData_[Lsnd_TRANSFORM], &rcvData_[Lrcv_MSR_TRANSFORM] , 12 * 8);
        snd_flag_joints_ = BYTE_LOW;

        std::lock_guard<std::mutex> lock2(rcvLock_);
        snd_transform_ = rcv_msr_transform_;
    }
    else
    {
        double Tr[12];
        snd_transform_.getData(Tr, Erl::RowMajor);
        copyToBytes( Tr, &sndData_[Lsnd_TRANSFORM], sizeof(Tr) );
    }

    copyToBytes( snd_joints_.data(),    &sndData_[Lsnd_JOINTS],    7*8 );
    copyToBytes( snd_stiffness_.data(), &sndData_[Lsnd_STIFFNESS], 7*8 );
    copyToBytes( snd_damping_.data(),   &sndData_[Lsnd_DAMPING],   7*8 );

    sndData_[Lsnd_F_ENABLE]    = snd_flags_enable_;
    sndData_[Lsnd_F_JOINTS]    = snd_flag_joints_;
    sndData_[Lsnd_F_IMPEDANCE] = snd_flags_enable_;
}

void KukaSunrise::updateMeasuredData()
{
    std::lock_guard<std::mutex> lock(rcvLock_);

    int64_t newTimestamp;
    copyFromBytes( &rcvData_[Lrcv_TIMESTAMP], &newTimestamp, sizeof(newTimestamp) );

    if (newTimestamp <= rcv_timestamp_)
        return;
    rcv_timestamp_ = newTimestamp;

    double msr_Tr[12];
    double dest_Tr[12];
    copyFromBytes( &rcvData_[Lrcv_MSR_TRANSFORM], msr_Tr,  sizeof(msr_Tr)  );
    copyFromBytes( &rcvData_[Lrcv_CMD_TRANSFORM], dest_Tr, sizeof(dest_Tr) );
    rcv_msr_transform_  = Erl::Transformd::fromPointer(msr_Tr, Erl::RowMajor);
    rcv_dest_transform_ = Erl::Transformd::fromPointer(dest_Tr, Erl::RowMajor);

    copyFromBytes( &rcvData_[Lrcv_JOINTS],    rcv_joints_.data(),    7*8 );
    copyFromBytes( &rcvData_[Lrcv_STIFFNESS], rcv_stiffness_.data(), 7*8 );
    copyFromBytes( &rcvData_[Lrcv_DAMPING],   rcv_damping_.data(),   7*8 );

    rcv_flag_impedance_ = rcvData_[Lrcv_F_IMPEDANCE];
}

bool KukaSunrise::close()
{
    if (isConnected())
    {
        commRunning_.store(false);
        commThread_.join();
        bool connecClose = connection_.close();

        snd_timestamp_ =  0;
        rcv_timestamp_ = -1;
        loopBackMsrTransform_ = false;
        return closedOK_ && connecClose;
    }

    return true;
}

bool KukaSunrise::isConnected() const
{
    return commRunning_.load();
}

void KukaSunrise::moveToDefaultPosition()
{
    double totalTime = 5000.0;
    Eigen::Matrix<double, 7, 1> currJoints = getJoints();
    Eigen::Matrix<double, 7, 1> destJoints;
    destJoints << M_PI/2, 0, 0, M_PI/2, 0, -M_PI/2, 0;

    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> t = std::chrono::high_resolution_clock::now() - start;

    while (t.count() <= totalTime)
    {
        double tt = t.count() / totalTime;

        Eigen::Matrix<double, 7, 1> newJoints;
        for (int i = 0; i < 7; i++)
            newJoints[i] = currJoints[i] + (destJoints[i] - currJoints[i])*(10*std::pow(tt, 3) - 15*std::pow(tt, 4) + 6*std::pow(tt, 5));
        setJoints(newJoints);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        t = std::chrono::high_resolution_clock::now() - start;
    }
    setJoints(destJoints);
}

void KukaSunrise::moveToPose(const Erl::Transformd& Pose, const double& time_ms)
{
    Erl::Timer_ms timer(time_ms);
    Erl::Transformd currPose = getDestTransform();

    while (!timer.expired())
    {
        Erl::Transformd newPose = Erl::Transformd::minJerkInterpolation(currPose, Pose, timer.elapsedBoundRatio());
        setTransform(newPose);
        Erl::sleep_ms(1);
    }
}
