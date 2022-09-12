enum RMLResultValue {
  //! \details
  //! The On-Line Trajectory Generation algorithm is working; the final
  //! state of motion has not been reached yet.
  RML_WORKING = 0,
  //! \details
  //! The desired final state of motion has been reached.
  RML_FINAL_STATE_REACHED = 1,
  //! \details
  //! This is the initialization value of TypeIVRMLPosition::ReturnValue
  //! and TypeIVRMLVelocity::ReturnValue. In practice, this value.
  //! cannot be returned
  RML_ERROR = -1,
  //! \details
  //! The applied input values are invalid (cf.
  //! RMLPositionInputParameters::CheckForValidity()
  //! RMLVelocityInputParameters::CheckForValidity()).
  RML_ERROR_INVALID_INPUT_VALUES = -100,
  //! \details
  //! An error occurred during the first step of
  //! the algorithm (i.e., during the calculation of the synchronization time).
  RML_ERROR_EXECUTION_TIME_CALCULATION = -101,
  //! \details
  //! An error occurred during the second step of
  //! the algorithm (i.e., during the synchronization of the trajectory).
  RML_ERROR_SYNCHRONIZATION = -102,
  //! \details
  //! The number of degree of freedom of th input parameters, the
  //! output parameters, and the On-Line Trajectory Generation
  //! algorithm do not match.
  RML_ERROR_NUMBER_OF_DOFS = -103,
  //! \details
  //! If the input flag RMLFlags::ONLY_PHASE_SYNCHRONIZATION is set
  //! and it is not possible to calculate a physically (and
  //! mathematically) correct phase-synchronized (i.e., homothetic)
  //! trajectory, this error value will be returned. Please note:
  //! Even if this error message is returned, feasible, steady, and
  //! continuous output values will be computed in \em any case.
  RML_ERROR_NO_PHASE_SYNCHRONIZATION = -104,
  //! \details
  //! If one of the pointers to objects of the classes
  //!
  //! - RMLPositionInputParameters / RMLVelocityInputParameters
  //! - RMLPositionOutputParameters / RMLVelocityOutputParameters
  //! - RMLPositionFlags / RMLVelocityFlags
  //!
  //! is NULL, this error value will be returned.
  RML_ERROR_NULL_POINTER = -105,
  //! \details
  //! To ensure numerical stability, the value of the minimum trajectory
  //! execution time is limited to a value of RML_MAX_EXECUTION_TIME
  //! (\f$ 10^{10} \f$ seconds).
  RML_ERROR_EXECUTION_TIME_TOO_BIG = -106,
  //! \details
  //! If either
  //! <ul>
  //!   <li>the method ReflexxesAPI::RMLPositionAtAGivenSampleTime() or</li>
  //!   <li>the method ReflexxesAPI::RMLVelocityAtAGivenSampleTime()</li>
  //! </ul>
  //! was used, the value of the parameter is negative or larger than
  //! the value of <c>RML_MAX_EXECUTION_TIME</c> (\f$10^{10}\,s\f$).
  RML_ERROR_USER_TIME_OUT_OF_RANGE = -107
};