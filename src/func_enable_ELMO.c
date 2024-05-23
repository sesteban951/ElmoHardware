/*
 * sfuntmpl_basic.c: Basic 'C' template for a level 2 S-function.
 *
 * Copyright 1990-2013 The MathWorks, Inc.
 */


/*
 * You must specify the S_FUNCTION_NAME as the name of your S-function
 * (i.e. replace sfuntmpl_basic with the name of your S-function).
 */

#define S_FUNCTION_NAME  func_enable_ELMO
#define S_FUNCTION_LEVEL 2

/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include "simstruc.h"
#include <stdio.h>

/* ELMO drive control 
 * -------------------
 * refer to DS-402 document from ELMO
 * refer to my notes
 */
void func_ELMO_ctrl(int_T driveNum, uint16_T statusWord, uint16_T *ctrlWord){
    
    uint16_T ctrlWord_tmp = 0; // disable voltage cmd
    char msg[255];
//     ctrlWord[0] = 0;
    
    if(!((statusWord >> 0) & 1) && !((statusWord >> 1) & 1) && !((statusWord >> 2) & 1) && !((statusWord >> 3) & 1) && !((statusWord >> 6) & 1)){    
        sprintf(msg,"Drive %d NOT ready!\n", driveNum);
        ssPrintf(msg);
        
    }else if(!((statusWord >> 0) & 1) && !((statusWord >> 1) & 1) && !((statusWord >> 2) & 1) && !((statusWord >> 3) & 1) && ((statusWord >> 6) & 1)){
        /* transition from "SOD" to "ready to switch on (RSO)" */
        sprintf(msg,"Drive %d in SOD mode.\n", driveNum);
        ssPrintf(msg);        
        ctrlWord_tmp = (1 << 1) | ctrlWord_tmp; 
        ctrlWord_tmp = (1 << 2) | ctrlWord_tmp;         
    }else if(((statusWord >> 0) & 1) && !((statusWord >> 1) & 1) && !((statusWord >> 2) & 1) && !((statusWord >> 3) & 1) && ((statusWord >> 5) & 1) && !((statusWord >> 6) & 1)){        
        sprintf(msg,"Drive %d in RSO mode.\n", driveNum);
        ssPrintf(msg);        
        /* transition from "RSO" to "switched on (SO)" */
        ctrlWord_tmp = (1 << 0) | ctrlWord_tmp; 
        ctrlWord_tmp = (1 << 1) | ctrlWord_tmp; 
        ctrlWord_tmp = (1 << 2) | ctrlWord_tmp; 
        
    }else if(((statusWord >> 0) & 1) && ((statusWord >> 1) & 1) && !((statusWord >> 2) & 1) && !((statusWord >> 3) & 1) && ((statusWord >> 5) & 1) && !((statusWord >> 6) & 1)){        
        sprintf(msg,"Drive %d in SO mode.\n", driveNum);
        ssPrintf(msg);
        /* transition from "SO" to "operation (OP)" mode */
        ctrlWord_tmp = (1 << 0) | ctrlWord_tmp; 
        ctrlWord_tmp = (1 << 1) | ctrlWord_tmp; 
        ctrlWord_tmp = (1 << 2) | ctrlWord_tmp; 
        ctrlWord_tmp = (1 << 3) | ctrlWord_tmp; 
        
    }else if(((statusWord >> 0) & 1) && ((statusWord >> 1) & 1) && ((statusWord >> 2) & 1) && !((statusWord >> 3) & 1) && ((statusWord >> 5) & 1) && !((statusWord >> 6) & 1)){     
        /* drive armed */  
        sprintf(msg,"ATTENTION!!! Drive %d ARMED.\n", driveNum);
        ssPrintf(msg);
        /* "OP" mode */
        ctrlWord_tmp = (1 << 0) | ctrlWord_tmp; 
        ctrlWord_tmp = (1 << 1) | ctrlWord_tmp; 
        ctrlWord_tmp = (1 << 2) | ctrlWord_tmp; 
        ctrlWord_tmp = (1 << 3) | ctrlWord_tmp;
        
    }else if(((statusWord >> 0) & 1) && ((statusWord >> 1) & 1) && ((statusWord >> 2) & 1) && !((statusWord >> 3) & 1) && !((statusWord >> 5) & 1) && !((statusWord >> 6) & 1)){
        /* drive failed */
        sprintf(msg,"Drive %d quick-stopped.\n", driveNum);
        ssPrintf(msg);
        
    }else if (((statusWord >> 0) & 1) && ((statusWord >> 1) & 1) && ((statusWord >> 2) & 1) && ((statusWord >> 3) & 1) && !((statusWord >> 6) & 1)){
        /* drive failed */
        sprintf(msg,"Drive %d FAILED: fault reaction active.\n", driveNum);
        ssPrintf(msg);
    
    }else if (!((statusWord >> 0) & 1) && !((statusWord >> 1) & 1) && !((statusWord >> 2) & 1) && ((statusWord >> 3) & 1) && !((statusWord >> 6) & 1)){
        /* drive failed */
        sprintf(msg,"Drive %d FAILED: fault.\n", driveNum);
        ssPrintf(msg);
    }else{
        /* unknown state */
        sprintf(msg,"Drive %d FAILED: unknown state.\n", driveNum);
        ssPrintf(msg);
    }
     
    ctrlWord[0] = ctrlWord_tmp;
}

/* ELMO drive shutdown
 * -------------------
 * When target is stopped shutdown all of the drives
 * refer to DS-402 document from ELMO
 * refer to my notes
 */
void func_ELMO_shutdown(int_T driveNum, uint16_T statusWord, uint16_T *ctrlWord){
    uint16_T ctrlWord_tmp = 0; // disable voltage cmd
    char msg[255];
    
    if(((statusWord >> 0) & 1) && ((statusWord >> 1) & 1) && ((statusWord >> 2) & 1) && !((statusWord >> 3) & 1) && ((statusWord >> 5) & 1) && !((statusWord >> 6) & 1)){
        sprintf(msg,"Drive %d is shutdown.\n", driveNum);
        ssPrintf(msg);    
    }else{
        sprintf(msg,"Drive %d was not ARMED.\n", driveNum);
        ssPrintf(msg);    
    }    
    
    ctrlWord[0] = ctrlWord_tmp;
}


/* Error handling
 * --------------
 *
 * You should use the following technique to report errors encountered within
 * an S-function:
 *
 *       ssSetErrorStatus(S,"Error encountered due to ...");
 *       return;
 *
 * Note that the 2nd argument to ssSetErrorStatus must be persistent memory.
 * It cannot be a local variable. For example the following will cause
 * unpredictable errors:
 *
 *      mdlOutputs()
 *      {
 *         char msg[256];         {ILLEGAL: to fix use "static char msg[256];"}
 *         sprintf(msg,"Error due to %s", string);
 *         ssSetErrorStatus(S,msg);
 *         return;
 *      }
 *
 */

/*====================*
 * S-function methods *
 *====================*/

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        /* Return if number of expected != number of actual parameters */
        return;
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, 7)) return;
    // input width
    ssSetInputPortWidth(S, 0, 6);
    ssSetInputPortWidth(S, 1, 6);
    ssSetInputPortWidth(S, 2, 6);
    ssSetInputPortWidth(S, 3, 6);
    ssSetInputPortWidth(S, 4, 7);
    ssSetInputPortWidth(S, 5, 3);
    ssSetInputPortWidth(S, 6, 1);  
    
    
   // input type
    ssSetInputPortDataType(S, 0, SS_UINT16);
    ssSetInputPortDataType(S, 1, SS_UINT16);
    ssSetInputPortDataType(S, 2, SS_INT32);
    ssSetInputPortDataType(S, 3, SS_INT32);
    ssSetInputPortDataType(S, 4, SS_UINT32);
    ssSetInputPortDataType(S, 5, SS_DOUBLE);
    ssSetInputPortDataType(S, 6, SS_DOUBLE);
    
    
    
    // direct input signal access
    ssSetInputPortRequiredContiguous(S, 0, true); /*direct input signal access*/
    ssSetInputPortRequiredContiguous(S, 1, true); /*direct input signal access*/
    ssSetInputPortRequiredContiguous(S, 2, true); /*direct input signal access*/
    ssSetInputPortRequiredContiguous(S, 3, true); /*direct input signal access*/
    ssSetInputPortRequiredContiguous(S, 4, true); /*direct input signal access*/
    ssSetInputPortRequiredContiguous(S, 5, true); /*direct input signal access*/
    ssSetInputPortRequiredContiguous(S, 6, true); /*direct input signal access*/
    /*
     * Set direct feedthrough flag (1=yes, 0=no).
     * A port has direct feedthrough if the input is used in either
     * the mdlOutputs or mdlGetTimeOfNextVarHit functions.
     */
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    ssSetInputPortDirectFeedThrough(S, 2, 1);
    ssSetInputPortDirectFeedThrough(S, 3, 1);
    ssSetInputPortDirectFeedThrough(S, 4, 1);
    ssSetInputPortDirectFeedThrough(S, 5, 1);
    ssSetInputPortDirectFeedThrough(S, 6, 1);

    if (!ssSetNumOutputPorts(S, 1)) return;
    // output width
    ssSetOutputPortWidth(S, 0, 6);
    
    // output type
    ssSetOutputPortDataType(S, 0, SS_UINT16);

    ssSetNumSampleTimes(S,PORT_BASED_SAMPLE_TIMES);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    /* Specify the sim state compliance to be same as a built-in block */
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

    ssSetOptions(S, 0);
}



/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    This function is used to specify the sample time(s) for your
 *    S-function. You must register the same number of sample times as
 *    specified in ssSetNumSampleTimes.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);

}



#define MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove function */
#if defined(MDL_INITIALIZE_CONDITIONS)
  /* Function: mdlInitializeConditions ========================================
   * Abstract:
   *    In this function, you should initialize the continuous and discrete
   *    states for your S-function block.  The initial states are placed
   *    in the state vector, ssGetContStates(S) or ssGetRealDiscStates(S).
   *    You can also perform any other initialization activities that your
   *    S-function may require. Note, this routine will be called at the
   *    start of simulation and if it is present in an enabled subsystem
   *    configured to reset states, it will be call when the enabled subsystem
   *    restarts execution to reset the states.
   */
  static void mdlInitializeConditions(SimStruct *S)
  {
  }
#endif /* MDL_INITIALIZE_CONDITIONS */



#define MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START) 
  /* Function: mdlStart =======================================================
   * Abstract:
   *    This function is called once at start of model execution. If you
   *    have states that should be initialized once, this is the place
   *    to do it.
   */
  static void mdlStart(SimStruct *S)
  {
  }
#endif /*  MDL_START */



/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    // pointer to input
    const uint16_T *u0 = (const uint16_T*) ssGetInputPortSignal(S,0); //SW
    const uint16_T *u1 = (const uint16_T*) ssGetInputPortSignal(S,1); //CW_disp
    const int32_T *u2 = (const int32_T*) ssGetInputPortSignal(S,2); //q
    const int32_T *u3 = (const int32_T*) ssGetInputPortSignal(S,3); //qd_disp
    const uint32_T *u4 = (const uint32_T*) ssGetInputPortSignal(S,4); //S_disp
    const real_T *u5 = (const real_T*) ssGetInputPortSignal(S,5); //SBG
    const real_T *u6 = (const real_T*) ssGetInputPortSignal(S,6); //t
    // pointer to output
    uint16_T *y = ssGetOutputPortSignal(S,0); //CW
    
    // local vars
    char msg[100];
    uint16_T ctrlWord;
    int_T i;
    int_T n;
    
    // SPECIFY MOTORS TO ENERGIZE BELOW:
    
    // Adding this in only to test specific joints, please remove later
    // Note: index for motor in C starts at 0, subtract 1 from etherCAT number for each motor
//     int_T j;
//     int_T active_motors[3] = {7, 9, 11};
//     for (i = 0; i < sizeof(active_motors)/sizeof(active_motors[0]); i++){
//         j = active_motors[i];
//         func_ELMO_ctrl(j, u0[j], &ctrlWord);
//         y[j] = ctrlWord;
//     }
     
    
    // enable ELMO drives
    for (i=0; i<6; i++){
        func_ELMO_ctrl(i, u0[i], &ctrlWord);
        y[i] = ctrlWord;
    }
    
    // report system status
       ssPrintf("\rExecution time: [%0.3f]\n", u6[0]);
    
//     sprintf(msg,"ELMO SW: [%d, %d, %d, %d]\n", u0[0], u0[1], u0[2], u0[3]);
//     ssPrintf(msg);
//     
//     sprintf(msg,"ELMO CW: [%d, %d, %d, %d]\n", y[0], y[1], y[2], y[3]);
//     ssPrintf(msg);
//     
//     sprintf(msg,"ELMO Enc: [%d, %d, %d, %d]\n", u2[0], u2[1], u2[2], u2[3]);
//     ssPrintf(msg);
//     
//     sprintf(msg,"ELMO Target Pos: [%d, %d, %d, %d]\n", u3[0], u3[1], u3[2], u3[3]);
//     ssPrintf(msg);
//     
//     sprintf(msg,"Leg Servo CMD: [%d]\n", u4[0]);
//     ssPrintf(msg);
//     
//     sprintf(msg,"Left Hip CMD: [%d]\n", u4[1]);
//     ssPrintf(msg);
//     
//     sprintf(msg,"Right Hip CMD: [%d]\n", u4[2]);
//     ssPrintf(msg);
//     
//     sprintf(msg,"BLDC CMD: [%d, %d, %d, %d]\n", u4[3], u4[4], u4[5], u4[6]);
//     ssPrintf(msg);
//     
//     sprintf(msg,"SBG: [%4.2f, %4.2f, %4.2f]\n", u5[0], u5[1], u5[2]);
//     ssPrintf(msg);
    
    // trick to avoid floating text
//     for (i=1; i<15; i++){
//         sprintf(msg,"\b\r");
//         ssPrintf(msg);
//     }
    
}



#define MDL_UPDATE  /* Change to #undef to remove function */
#if defined(MDL_UPDATE)
  /* Function: mdlUpdate ======================================================
   * Abstract:
   *    This function is called once for every major integration time step.
   *    Discrete states are typically updated here, but this function is useful
   *    for performing any tasks that should only take place once per
   *    integration step.
   */
  static void mdlUpdate(SimStruct *S, int_T tid)
  {
  }
#endif /* MDL_UPDATE */



#define MDL_DERIVATIVES  /* Change to #undef to remove function */
#if defined(MDL_DERIVATIVES)
  /* Function: mdlDerivatives =================================================
   * Abstract:
   *    In this function, you compute the S-function block's derivatives.
   *    The derivatives are placed in the derivative vector, ssGetdX(S).
   */
  static void mdlDerivatives(SimStruct *S)
  {
  }
#endif /* MDL_DERIVATIVES */



/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
    const uint16_T *u0 = (const uint16_T*) ssGetInputPortSignal(S,0); //SW
    uint16_T *y = ssGetOutputPortSignal(S,0); //CW
    uint16_T ctrlWord;
    int_T i;
    
    // disable ELMO drives
//     for (i=0; i<4; i++){
//         func_ELMO_shutdown(i, u0[i], &ctrlWord);
//         y[i] = ctrlWord;
//     }
    
}


/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
