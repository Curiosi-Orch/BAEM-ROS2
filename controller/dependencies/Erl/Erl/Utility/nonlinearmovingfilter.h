/*
 * Copyright (c) 2013-2017 Konrad Leibrandt <konrad.lei@gmx.de>
 * Copyright (c) 2013-2017 Gauthier Gras <gauthier.gras@gmail.com>
 * Licensed under the MIT license. See the license file LICENSE.
*/

#ifndef ERL_NONLINEARMOVINGFILTER_H
#define ERL_NONLINEARMOVINGFILTER_H

#include <Eigen/Core>

namespace Erl{
//Eigen::FullPivHouseholderQR<Eigen::Matrix<T,Eigen::Dynamic,PolyDegree+1,Eigen::RowMajor> >
template<class T, int DataDoF, int PolyDegree, class Solver = Eigen::JacobiSVD<Eigen::Matrix<T,Eigen::Dynamic,PolyDegree+1,Eigen::RowMajor> > >
class NonlinearMovingFilter
{
public:
    NonlinearMovingFilter(unsigned _FilterSize, Eigen::Matrix<T,DataDoF,1> _Initialization, T _InitTimeSpacing, T _InitTime)
        : m_FilterSize(_FilterSize)
        , m_FilterRow(0)
    {
        m_FilterTimeData = Eigen::Matrix<T,Eigen::Dynamic,PolyDegree+1,Eigen::RowMajor>(_FilterSize,PolyDegree+1);

        Erl::static_for<0,DataDoF,1>()([&](int i)
        {
            m_FilterValueData[i] = _Initialization(i)*Eigen::Matrix<T,Eigen::Dynamic,1,Eigen::RowMajor>::Ones(_FilterSize,1);
        });
        for(unsigned i(0);i<m_FilterSize;i++)
        {
            applyTimeVector(_InitTime-T(i+1)*_InitTimeSpacing,i);
        }
        calcPolyCoeff();
    }

    ~NonlinearMovingFilter(){}

    void add(const Eigen::Matrix<T,DataDoF,1>& _element, const T& _timestamp)
    {
        applyTimeVector(_timestamp,m_FilterRow);
        Erl::static_for<0,DataDoF,1>()([&](int i)
        {
            m_FilterValueData[i].row(m_FilterRow) << _element(i);
        });

        calcPolyCoeff();
        m_FilterRow++;
        if(m_FilterRow==m_FilterSize){m_FilterRow=0;}
    }

    Eigen::Matrix<T,DataDoF,1> get(const T& _time)
    {
        Eigen::Matrix<T,DataDoF,1>  c_ReturnVec;
        Eigen::Matrix<T,PolyDegree+1,1> c_TimeVec;
        createTimeVector(_time,c_TimeVec);

        Erl::static_for<0,DataDoF,1>()([&](int i)
        {
            c_ReturnVec.row(i) = m_PolyCoeff[i]*c_TimeVec;
        });
        return c_ReturnVec;
    }



public:
    Eigen::Matrix<T,Eigen::Dynamic,PolyDegree+1,Eigen::RowMajor> m_FilterTimeData;
    Eigen::Matrix<T,Eigen::Dynamic,1>                            m_FilterValueData[DataDoF];
    unsigned                                                     m_FilterSize;
    unsigned                                                     m_FilterRow;
    Eigen::Matrix<T,1,PolyDegree+1,Eigen::RowMajor>              m_PolyCoeff[DataDoF];

    inline void createTimeVector(T _time, typename Eigen::MatrixBase<Eigen::Matrix<T,Eigen::Dynamic,PolyDegree+1,Eigen::RowMajor> >::RowXpr _Vec)
    {
        _Vec(PolyDegree) = T(1);
        Erl::static_for<PolyDegree-1,-1,-1>()([&](int i)
        {
            _Vec(i) = _time*_Vec(i+1);
        });
    }
    inline void createTimeVector(T _time, Eigen::Matrix<T,PolyDegree+1,1> & _Vec)
    {
        _Vec(PolyDegree) = T(1);
        Erl::static_for<PolyDegree-1,-1,-1>()([&](int i)
        {
            _Vec(i) = _time*_Vec(i+1);
        });
    }
    inline void applyTimeVector(T _time, unsigned _row)
    {
        createTimeVector(_time,m_FilterTimeData.row(_row));
    }
    inline void calcPolyCoeff()
    {
        // Eigen::JacobiSVD<Eigen::Matrix<T,Eigen::Dynamic,PolyDegree+1,Eigen::RowMajor> > c_TimeMatrixDecomp(m_FilterTimeData,Eigen::ComputeFullU | Eigen::ComputeFullV);
        // Eigen::FullPivHouseholderQR<Eigen::Matrix<T,Eigen::Dynamic,PolyDegree+1,Eigen::RowMajor> > c_TimeMatrixDecomp;

        Solver c_TimeMatrixDecomp;

        c_TimeMatrixDecomp.compute(m_FilterTimeData, Eigen::ComputeFullU | Eigen::ComputeFullV);

        Erl::static_for<0,DataDoF,1>()([&](int i)
        {
            m_PolyCoeff[i]= c_TimeMatrixDecomp.solve(m_FilterValueData[i]).transpose();
        });
    }

};

//Eigen::FullPivHouseholderQR<Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> >
//Eigen::JacobiSVD<Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> >
template<class T, int DataDoF, int PolyDegree, class Solver = Eigen::JacobiSVD<Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> > >
class NonlinearPeriodicMovingFilter
{
public:
    NonlinearPeriodicMovingFilter(unsigned _FilterSize, Eigen::Matrix<T,DataDoF,1> _Initialization, T _InitTimeSpacing, T _InitTime,
                                  T _ProbCutoff = 0.05, unsigned _ofac=4, unsigned _hifac=2)
        : m_FilterSize(_FilterSize)
        , m_OFAC(_ofac)
        , m_HIFAC(_hifac)
        , m_ProbCutoff(_ProbCutoff)
    {
        m_FilterTimeData          = Eigen::Matrix<T,Eigen::Dynamic,1>(m_FilterSize,1);
        for(unsigned i(0);i<m_FilterSize;i++)
        {
            m_FilterTimeData(i) = _InitTime-T(i+1)*_InitTimeSpacing;
        }

        Erl::static_for<0,DataDoF,1>()([&](int i)
        {
            m_FilterValueData[i]         = _Initialization(i)*Eigen::Matrix<T,Eigen::Dynamic,1>::Ones(m_FilterSize,1);
            //m_FilterValueDataWindowed[i] = Eigen::Matrix<T,Eigen::Dynamic,1>(m_FilterSize,1);
        });

        //createWindow();

        m_NFrequ =0.5*m_OFAC*m_HIFAC*m_FilterSize;

        m_wi             = Eigen::Matrix<T,Eigen::Dynamic,1>(m_FilterSize,1);
        m_wpi            = Eigen::Matrix<T,Eigen::Dynamic,1>(m_FilterSize,1);
        m_wpr            = Eigen::Matrix<T,Eigen::Dynamic,1>(m_FilterSize,1);
        m_wr             = Eigen::Matrix<T,Eigen::Dynamic,1>(m_FilterSize,1);
        m_Frequency      = Eigen::Matrix<T,Eigen::Dynamic,1>(m_NFrequ,1);
        m_FrequencyPower = Eigen::Matrix<T,Eigen::Dynamic,1>(m_NFrequ,1);

        m_TmpOptiData    = Eigen::Matrix<T,Eigen::Dynamic,(2*PolyDegree)+1>(m_FilterSize,(2*PolyDegree)+1);
        m_TmpFrequData   = Eigen::Matrix<T,Eigen::Dynamic,1>(m_FilterSize,1);

        Erl::static_for<0,DataDoF,1>()([&](int c_DataIdx)
        {
                lomb(c_DataIdx);
        });
        Erl::static_for<0,DataDoF,1>()([&](int c_DataIdx)
        {
                calcFunctionCoeff(c_DataIdx);
        });
    }

    ~NonlinearPeriodicMovingFilter(){}

    void add(const Eigen::Matrix<T,DataDoF,1>& _element, const T& _timestamp)
    {
        memmove(m_FilterTimeData.data()+1,m_FilterTimeData.data(),sizeof(T)*(m_FilterSize-1));
        Erl::static_for<0,DataDoF,1>()([&](int i)
        {
            memmove(m_FilterValueData[i].data()+1,m_FilterValueData[i].data(),sizeof(T)*(m_FilterSize-1));
        });
        m_FilterTimeData(0) = _timestamp;
        Erl::static_for<0,DataDoF,1>()([&](int i)
        {
            m_FilterValueData[i](0) = _element(i);
        });
        Erl::static_for<0,DataDoF,1>()([&](int c_DataIdx)
        {
                lomb(c_DataIdx);
        });
        Erl::static_for<0,DataDoF,1>()([&](int c_DataIdx)
        {
                calcFunctionCoeff(c_DataIdx);
        });

    }

    Eigen::Matrix<T,DataDoF,1> get(const T& _time)
    {
        Eigen::Matrix<T,DataDoF,1>        c_ReturnVec;
        Eigen::Matrix<T,2*PolyDegree+1,1> c_FrequVec;
        // createTimeVector(_time,c_TimeVec);

        Erl::static_for<0,DataDoF,1>()([&](int c_DataIdx)
        {
            c_FrequVec(0) = T(1);

            for(unsigned i(0);i<m_NumHarmonics[c_DataIdx];i++)
            {
                T c_Frequency = 2*M_PI*m_MaxFrequencyPowerFrequency[c_DataIdx][PolyDegree-i-1]*_time;

                c_FrequVec(1+2*i) = std::cos(c_Frequency) ;
                c_FrequVec(2+2*i) = std::sin(c_Frequency) ;
            }

            c_ReturnVec.row(c_DataIdx) =  m_FuncCoeff[c_DataIdx].leftCols(1+2*m_NumHarmonics[c_DataIdx])
                                        * c_FrequVec.topRows(1+2*m_NumHarmonics[c_DataIdx]);

        });
        return c_ReturnVec;
    }



public:
    Eigen::Matrix<T,Eigen::Dynamic,1>   m_FilterTimeData;
    Eigen::Matrix<T,Eigen::Dynamic,1>   m_FilterValueData[DataDoF];
    Eigen::Matrix<T,Eigen::Dynamic,1>   m_FilterValueDataWindowed[DataDoF];
    Eigen::Matrix<T,Eigen::Dynamic,1>   m_FilterWindow;
    unsigned                            m_FilterSize;
    unsigned                            m_OFAC;
    unsigned                            m_HIFAC;
    Eigen::Matrix<T,Eigen::Dynamic,1>   m_Frequency;
    Eigen::Matrix<T,Eigen::Dynamic,1>   m_FrequencyPower;
    unsigned                            m_NFrequ;
    T                                   m_ProbCutoff;

    Eigen::Matrix<T,1,PolyDegree>       m_MaxFrequencyPower[DataDoF];
    Eigen::Matrix<T,1,PolyDegree>       m_MaxFrequencyPowerProb[DataDoF];
    Eigen::Matrix<T,1,PolyDegree>       m_MaxFrequencyPowerFrequency[DataDoF];
    unsigned                            m_NumHarmonics[DataDoF];

    Eigen::Matrix<T,1,(2*PolyDegree)+1> m_FuncCoeff[DataDoF];

    Eigen::Matrix<T,Eigen::Dynamic,(2*PolyDegree)+1,Eigen::ColMajor> m_TmpOptiData;
    Eigen::Matrix<T,Eigen::Dynamic,1>                                m_TmpFrequData;

    Eigen::Matrix<T,Eigen::Dynamic,1>    m_wi ;
    Eigen::Matrix<T,Eigen::Dynamic,1>    m_wpi;
    Eigen::Matrix<T,Eigen::Dynamic,1>    m_wpr;
    Eigen::Matrix<T,Eigen::Dynamic,1>    m_wr ;



    //    inline void lomb()
    //    {
    //        T c_TimeDiff = m_FilterTimeData(m_FilterSize-1) - m_FilterTimeData(0);
    //        Erl::static_for<0,DataDoF,1>()([&](int i)
    //        {
    //            m_Mu[i] = m_FilterValueData[i].mean();
    //        });
    //        Erl::static_for<0,DataDoF,1>()([&](int i)
    //        {
    //            m_Sigma[i] = (m_FilterValueData[i].array() - m_Mu[i]).matrix().squaredNorm()/T(m_FilterSize);
    //        });
    //        unsigned c_NFrequ = m_HIFAC*m_FilterSize*m_OFAC/2;
    //        m_Frequency.setLinSpaced(T(2*M_PI)/(c_TimeDiff*m_OFAC),T(2*M_PI)*m_HIFAC*m_FilterSize/(T(2)*c_TimeDiff),c_NFrequ);
    //        m_WtT = m_Frequency*m_FilterTimeData.transpose();
    //        m_SineTmp   = T(2)*(m_WtT).unaryExpr([](T e){return std::sin(e);}).rowwise().sum();
    //        m_CosineTmp = T(2)*(m_WtT).unaryExpr([](T e){return std::cos(e);}).rowwise().sum();
    //        for(unsigned i(0);i<c_NFrequ;i++)
    //        {
    //            m_Tau = std::atan2(m_SineTmp(i),m_CosineTmp(i))/2*m_Frequency(i);
    //        }
    //        Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic,1> c_cterm = (m_WtT.colwise() - (m_Frequency.array()* m_Tau.array()).matrix()).unaryExpr([](T e){return std::cos(e);});
    //        Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic,1> c_sterm = (m_WtT.colwise() - (m_Frequency.array()* m_Tau.array()).matrix()).unaryExpr([](T e){return std::sin(e);});
    //    }

    static inline T square(T const& _v){return _v*_v;}
    inline T windowFunctionHamming(unsigned n)
    {
        constexpr T s_alpha = 0.54;
        constexpr T s_beta  = 0.46;
        return (s_alpha - s_beta*std::cos( (T(2*M_PI) * n) / (m_FilterSize-1) ));
    }
    inline T windowFunctionRectangular(unsigned n)
    {
        return T(1.0);
    }
    inline void createWindow()
    {
        m_FilterWindow  = Eigen::Matrix<T,Eigen::Dynamic,1>(m_FilterSize,1);
        for(unsigned i(0);i<m_FilterSize;i++)
        {
            m_FilterWindow(i) = windowFunctionHamming(i);
        }
    }
    inline void windowData(unsigned _DataIdx)
    {

        m_FilterValueDataWindowed[_DataIdx] = m_FilterValueData[_DataIdx].array()*m_FilterWindow.array();

    }
    inline void lomb(unsigned _DataIdx)
    {
        //windowData(_DataIdx);

        T c_Mu  =  m_FilterValueData[_DataIdx].mean();
        T c_Var = (m_FilterValueData[_DataIdx].array() - c_Mu).matrix().squaredNorm()/T(m_FilterSize);

        T c,cc,cwtau,pnow,s,ss,sumc,sumcy,sums,sumsh,
                sumsy,swtau,wtau,xave,xdif,xmax,xmin,yy;

        T arg,wtemp,*c_wi,*c_wpi,*c_wpr,*c_wr;
        c_wi =m_wi.data();
        c_wpi=m_wpi.data();
        c_wpr=m_wpr.data();
        c_wr =m_wr.data();

        T    c_FrequencyPowerPrev=0;
        bool c_FrequencyPowerInc = true;




        xmax = m_FilterTimeData[0];  // ; Go through data to get the range of abscisas.
        xmin = m_FilterTimeData[m_FilterSize-1];
        xdif = xmax-xmin;
        xave=T(0.5)*(xmax+xmin);

        m_MaxFrequencyPower[_DataIdx].setZero();
        m_MaxFrequencyPowerFrequency[_DataIdx].setZero();

        pnow=T(1.0)/(xdif*m_OFAC);                         // Starting frequency.
        for (unsigned j=0;j<m_FilterSize;j++)              //Initialize values for the trigonometric recurrences
        {
            arg=2*M_PI*((m_FilterTimeData[j]-xave)*pnow);  //at each data point. The recurrences
            c_wpr[j] =  -2.0*square(std::sin(0.5*arg));         //are done in double precision.
            c_wpi[j] = std::sin(arg);
            c_wr [j] = std::cos(arg);
            c_wi [j] = c_wpi[j];
        }


        for (unsigned i=0;i<(m_NFrequ);i++)
        {              //Main loop over the frequencies to be evaluated.
            m_Frequency[i]=pnow;
            sumsh=sumc=0.0;                         // First, loop over the data to get t and related quantities.
            for (unsigned j=0;j<m_FilterSize;j++)
            {
                c=c_wr[j];
                s=c_wi[j];
                sumsh += s*c;
                sumc += (c-s)*(c+s);
            }
            wtau=0.5*std::atan2(2.0*sumsh,sumc);
            swtau=std::sin(wtau);
            cwtau=std::cos(wtau);
            sums=sumc=sumsy=sumcy=0.0;              //Then, loop over the data again to get the periodogram value.
            for (unsigned j=0;j<m_FilterSize;j++)
            {
                s=c_wi[j];
                c=c_wr[j];
                ss=s*cwtau-c*swtau;
                cc=c*cwtau+s*swtau;
                sums += ss*ss;
                sumc += cc*cc;
                yy=m_FilterValueData[_DataIdx][j]-c_Mu;
                sumsy += yy*ss;
                sumcy += yy*cc;
                c_wr[j]=((wtemp=c_wr[j])*c_wpr[j]-c_wi[j]*c_wpi[j])+c_wr[j];        //     Update the trigonometric recurrences.
                c_wi[j]=(c_wi[j]*c_wpr[j]+wtemp*c_wpi[j])+c_wi[j];
            }

            m_FrequencyPower[i]=0.5*(sumcy*sumcy/sumc+sumsy*sumsy/sums)/c_Var;

            if( (m_FrequencyPower[i]<c_FrequencyPowerPrev)) // Frequency Power Reduces
            {
                if(c_FrequencyPowerInc) // If it increase before - We were at the tip.
                {
                    if(c_FrequencyPowerPrev>m_MaxFrequencyPower[_DataIdx][0])
                    {
                        unsigned k = 1;
                        while((k<PolyDegree) && (c_FrequencyPowerPrev>m_MaxFrequencyPower[_DataIdx][k]) ){k++;}
                        k--;
                        if(k>0)
                        {
                            memmove(&m_MaxFrequencyPower[_DataIdx][0],&m_MaxFrequencyPower[_DataIdx][1] ,sizeof(T)*k);
                            memmove(&m_MaxFrequencyPowerFrequency[_DataIdx][0] ,&m_MaxFrequencyPowerFrequency[_DataIdx][1]  ,sizeof(T)*k);
                        }
                        m_MaxFrequencyPower[_DataIdx][k]           = c_FrequencyPowerPrev;
                        m_MaxFrequencyPowerFrequency[_DataIdx][k]  = m_Frequency[i-1];
                    }
                }
                c_FrequencyPowerInc = false;
            }
            else
            {
                c_FrequencyPowerInc = true;
            }



            pnow += 1.0/(m_OFAC*xdif);                                // The next frequency.
            c_FrequencyPowerPrev = m_FrequencyPower[i];
        }                                                           // Evaluate statistical significance of the maximum.
        m_NumHarmonics[_DataIdx] = 0;
        Erl::static_for<0,PolyDegree,1>()([&](int i)
        {
            T c_expy = std::exp(-m_MaxFrequencyPower[_DataIdx][i]);
            T c_prob = 2.0*(m_NFrequ)/m_OFAC * c_expy;
            if (c_prob > 0.01){c_prob=1.0-std::pow(1.0-c_expy,2.0*(m_NFrequ)/m_OFAC);}

            m_MaxFrequencyPowerProb[_DataIdx][i] = c_prob;
            if(c_prob< m_ProbCutoff){m_NumHarmonics[_DataIdx]++;}

        });
        // CALCULATE THE FACTOR
        if(m_NumHarmonics[_DataIdx]!=0)
        {
            //m_MaxFrequencyPowerFrequency[_DataIdx]  = m_MaxFrequencyPowerFrequency[_DataIdx] *(1.0/(2.0* m_MaxFrequencyPowerFrequency[_DataIdx][m_NumHarmonics[_DataIdx]-1]) );
        }



        //m_MaxFrequencyPowerProb[_DataIdx]= prob;
    }
    inline void calcFunctionCoeff(unsigned _DataIdx)
    {
        m_TmpOptiData.col(0).setOnes();

        std::cout<<m_MaxFrequencyPowerFrequency[_DataIdx]<<"||"<<m_MaxFrequencyPower[_DataIdx]<<std::endl;

        for(unsigned i(0);i<m_NumHarmonics[_DataIdx];i++)
        {
            T c_Frequency = m_MaxFrequencyPowerFrequency[_DataIdx][PolyDegree-i-1];

            m_TmpFrequData = 2*M_PI*c_Frequency*m_FilterTimeData;

            m_TmpOptiData.col(1+(i*2)) = m_TmpFrequData.unaryExpr([](T e){return std::cos(e);});
            m_TmpOptiData.col(2+(i*2)) = m_TmpFrequData.unaryExpr([](T e){return std::sin(e);});
        }

        Solver c_TimeMatrixDecomp(m_TmpOptiData.leftCols(1+2*m_NumHarmonics[_DataIdx]), Eigen::ComputeFullU | Eigen::ComputeFullV);

        m_FuncCoeff[_DataIdx].leftCols(1+2*m_NumHarmonics[_DataIdx]) = c_TimeMatrixDecomp.solve(m_FilterValueData[_DataIdx]).transpose();

    }

    static inline T lomb_static(unsigned _DataIdx, const Eigen::Matrix<T,Eigen::Dynamic,1>& _Data, const Eigen::Matrix<T,Eigen::Dynamic,1>& _Time,
                                   unsigned _OFAC, unsigned _HIFAC)
    {
        //windowData(_DataIdx);

        unsigned c_FilterSize = _Data.rows();
        assert(_Data.rows() == _Time.rows() && "Data and Time rows have to be equal size!");


        T c_NFrequ =0.5*_OFAC*_HIFAC*c_FilterSize;

        Eigen::Matrix<T,Eigen::Dynamic,1> c_wiD             = Eigen::Matrix<T,Eigen::Dynamic,1>(c_FilterSize,1);
        Eigen::Matrix<T,Eigen::Dynamic,1> c_wpiD            = Eigen::Matrix<T,Eigen::Dynamic,1>(c_FilterSize,1);
        Eigen::Matrix<T,Eigen::Dynamic,1> c_wprD            = Eigen::Matrix<T,Eigen::Dynamic,1>(c_FilterSize,1);
        Eigen::Matrix<T,Eigen::Dynamic,1> c_wrD             = Eigen::Matrix<T,Eigen::Dynamic,1>(c_FilterSize,1);
        Eigen::Matrix<T,Eigen::Dynamic,1> c_FrequencyD      = Eigen::Matrix<T,Eigen::Dynamic,1>(c_NFrequ,1);
        Eigen::Matrix<T,Eigen::Dynamic,1> c_FrequencyPowerD = Eigen::Matrix<T,Eigen::Dynamic,1>(c_NFrequ,1);

        T c_Mu  =  _Data.mean();
        T c_Var = (_Data.array() - c_Mu).matrix().squaredNorm()/T(c_FilterSize);

        T c,cc,cwtau,pnow,s,ss,sumc,sumcy,sums,sumsh,
                sumsy,swtau,wtau,xave,xdif,xmax,xmin,yy;

        T arg,wtemp,*c_wi,*c_wpi,*c_wpr,*c_wr;
        c_wi =c_wiD.data();
        c_wpi=c_wpiD.data();
        c_wpr=c_wprD.data();
        c_wr =c_wrD.data();

        T    c_FrequencyPowerMax=0;
        T    c_FrequencyPowerFrequencyMax=0;




        xmax = _Time[0];  // ; Go through data to get the range of abscisas.
        xmin = _Time[c_FilterSize-1];
        xdif = xmax-xmin;
        xave=T(0.5)*(xmax+xmin);


        pnow=T(1.0)/(xdif*_OFAC);                         // Starting frequency.
        for (unsigned j=0;j<c_FilterSize;j++)              //Initialize values for the trigonometric recurrences
        {
            arg=2*M_PI*((_Time[j]-xave)*pnow);  //at each data point. The recurrences
            c_wpr[j] =  -2.0*square(std::sin(0.5*arg));         //are done in double precision.
            c_wpi[j] = std::sin(arg);
            c_wr [j] = std::cos(arg);
            c_wi [j] = c_wpi[j];
        }


        for (unsigned i=0;i<(c_NFrequ);i++)
        {              //Main loop over the frequencies to be evaluated.
            c_FrequencyD[i]=pnow;
            sumsh=sumc=0.0;                         // First, loop over the data to get t and related quantities.
            for (unsigned j=0;j<c_FilterSize;j++)
            {
                c=c_wr[j];
                s=c_wi[j];
                sumsh += s*c;
                sumc += (c-s)*(c+s);
            }
            wtau=0.5*std::atan2(2.0*sumsh,sumc);
            swtau=std::sin(wtau);
            cwtau=std::cos(wtau);
            sums=sumc=sumsy=sumcy=0.0;              //Then, loop over the data again to get the periodogram value.
            for (unsigned j=0;j<c_FilterSize;j++)
            {
                s=c_wi[j];
                c=c_wr[j];
                ss=s*cwtau-c*swtau;
                cc=c*cwtau+s*swtau;
                sums += ss*ss;
                sumc += cc*cc;
                yy=_Data[_DataIdx][j]-c_Mu;
                sumsy += yy*ss;
                sumcy += yy*cc;
                c_wr[j]=((wtemp=c_wr[j])*c_wpr[j]-c_wi[j]*c_wpi[j])+c_wr[j];        //     Update the trigonometric recurrences.
                c_wi[j]=(c_wi[j]*c_wpr[j]+wtemp*c_wpi[j])+c_wi[j];
            }

            c_FrequencyPowerD[i]=0.5*(sumcy*sumcy/sumc+sumsy*sumsy/sums)/c_Var;

            if( c_FrequencyPowerD[i]>c_FrequencyPowerMax) // Frequency Power Reduces
            {
                c_FrequencyPowerMax = c_FrequencyPowerD[i];
                c_FrequencyPowerFrequencyMax = c_FrequencyD[i];
            }
            pnow += 1.0/(_OFAC*xdif);                                // The next frequency.

        }                                                           // Evaluate statistical significance of the maximum.


        return c_FrequencyPowerFrequencyMax;

    }

    static T calculateDelay(const Eigen::Matrix<T,Eigen::Dynamic,1>& _SensorPrimary,
                            const Eigen::Matrix<T,Eigen::Dynamic,1>& _SensorSecondary,
                            const Eigen::Matrix<T,Eigen::Dynamic,1>& _Time,
                            const T& _MaxDelayTime)
    {
        unsigned c_Size = _SensorPrimary.rows();

        assert(_SensorPrimary.rows() == _SensorSecondary.rows() &&
               _SensorPrimary.rows() == _Time.rows() &&
               "Number of Data Points has to be equal for all Datavectors.");

        T c_DeltaT      = _Time.maxCoeff() - _Time.minCoeff();
        T c_AveTimeStep = c_DeltaT / T(_Time.rows());
        unsigned c_MaxDelaySteps = _MaxDelayTime/c_AveTimeStep;

        Eigen::Matrix<T,Eigen::Dynamic,1> c_SensorPrimary   = _SensorPrimary.array()   - _SensorPrimary.mean();
        Eigen::Matrix<T,Eigen::Dynamic,1> c_SensorSecondary = _SensorSecondary.array() - _SensorSecondary.mean();

        unsigned c_BestIter;
        T c_CostBest   = std::numeric_limits<T>::max();
        T c_CostCurr   = std::numeric_limits<T>::max();

        for(unsigned i(0);i<c_MaxDelaySteps;i++)
        {
            c_CostCurr = (c_SensorPrimary.bottomRows(c_Size-i) - c_SensorSecondary.topRows(c_Size-i)).squaredNorm();
            if(c_CostCurr<c_CostBest)
            {
                c_CostBest     = c_CostCurr;
                c_BestIter     = i;
            }
        }
        return c_BestIter * c_AveTimeStep;
    }
};
}

#endif // ERL_NONLINEARMOVINGFILTER_H

