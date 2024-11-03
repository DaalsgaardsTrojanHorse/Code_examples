// Exam - Real Time Systems and Data Collection
#include <krnl.h>

// Typedefs for convenience
typedef uint8_t  U8;
typedef uint16_t U16;
typedef uint32_t U32;
typedef uint64_t U64;
typedef int8_t   I8;
typedef int16_t  I16;
typedef int32_t  I32;
typedef int64_t  I64;

// Comment this #define out if you want to see log of regulator task (read speed and speed setpoint)
#define SUPPRESS_REGULATOR_LOG
// Comment this #define in if you want to log as much information as possible (eg. quad counter and time of each regulator loop)
//#define LOG_ALL

// Size of task stacks
#define STACKSIZE 200

// Size of log msg queue
// Number of logMsg structs in queue
#define LOG_MSG_Q_LEN 10

// Analog pin for speed ref
#define SPEEDREF_PIN A0

// Quad encoder pins
// INT0 is used. Interrupt pin is pin 2.
#define PIND_QUAD_SIGN_PIN 1 // PORTB, pin 9 on arduino

// H-bridge PWM pins
// PORTB
#define H_BRIDGE_PWM 2
#define H_BRIDGE_DIR 4
#define H_BRIDGE_PWM_NUMERIC H_BRIDGE_PWM+8 // PWM pin in arduino pin naming convention

// Safety maximum values
// Here so the motor won't vibrate itself to pieces.
// Maximum duty cycle of the PWM signal
const U8 pwmMaxDc = 100;

// Protocol definition
//#define PROTO_STARTCONDITION   0x02
//#define PROTO_STOPCONDITION    0x03
#define PROTO_STARTCONDITION   'A' // easier for testing
#define PROTO_STOPCONDITION    'B' // easier for testing
#define PROTO_ESCAPECONDITION  0x10
#define PROTO_PIDUPDATE_PREFIX 'U'
#define PROTO_LOG_PREFIX       'L'

// Log message prefixes and ids
// Must be chars
#define LOG_REGULATOR_ID   'R'
#define LOG_SERIAL_ID      'D' // 'D' for debug panel
#define LOG_TIME_PREFIX    'T'
#define LOG_COUNTER_PREFIX 'C'
#define LOG_SPEED_PREFIX   'S'
#define LOG_PID_PREFIX     'P'

// Task priority
const int PIDUPDATE_PRIO   = 13;
const int LOG_PRIO         = 11;
const int REGULATOR_PRIO   = 10;
const int SPEEDREF_PRIO    = 12;

// Period of periodical tasks
const int SPEEDREF_PERIOD  = 100;
const int CONTROL_PERIOD   = 2;
const int LOG_PERIOD       = 2;
const int PIDUPDATE_PERIOD = 500;

// Timeout for shared data mutex semaphores
const int SPEEDREF_TIMEOUT  = 10;
const int PID_TIMEOUT       = 10;
const int QUADSPEED_TIMEOUT = 0;

// Debug pins pulled high when task is running
#define PIDUPDATE_DBG_PIN 4
#define LOG_DBG_PIN       5
#define CONTROL_DBG_PIN   6
#define SPEEDREF_DBG_PIN  7

// utility macro to set bit n of a U8 to either 1 or 0
#define setBitNToX(target, n, x) target ^= (-(x) ^ target) & (1 << n)

// Struct for sending new PID params through UART
// Size must be 12 bytes in order to ensure same-length UART protocol
struct pidParamsTp {
    double P;
    double I;
    double D;
};
// Maximum length of a layer 3 packet
// PID parameters are sent as strings delimited by ','
// e.x. "13.6,12.0,21.42"
// On average maximum 5 bytes per parameter is legal to prevent overflow
// This is a somewhat arbitrarily chosen limit
// Maximum length will be defined as 
#define MAX_BYTES_PER_PID_PARAM 8
const int MAXLEN_LAYER3 = 1 + 3*MAX_BYTES_PER_PID_PARAM + 2;

// Struct for sending log messages
// Size must be 12 bytes in order to ensure same-length UART protocol
#define LOGMSG_DATA_LEN sizeof(pidParamsTp)
struct logMsg {
    U8 id;
    U8 logType;
    U8 data[LOGMSG_DATA_LEN];
};

// Speed reference calculation coefficient, k
// vref = k * potmetervalue
const double SPEEDREF_COEFF = 20.0 * 1.0/1023;

struct k_t *pTaskSpeedRef;
struct k_t *pTaskRegulator;
struct k_t *pTaskLog;
struct k_t *pTaskPIDUpdate;

struct k_t *timerSemSpeedRef;
struct k_t *timerSemControl;
struct k_t *timerSemPIDUpdate;
struct k_t *timerSemLog;

struct k_t *mutSemSpeedRef;
struct k_t *mutSemPID;
struct k_t *mutSemQuadCtr;

// log message queue between regulator and log task
struct k_msg_t *logMsgQ;
struct logMsg logDataBufForMsgQ[LOG_MSG_Q_LEN];

U8 stackSpeedRef[STACKSIZE];
U8 stackRegulator[STACKSIZE];
U8 stackLog[STACKSIZE];
U8 stackPIDUpdate[STACKSIZE];

void speedRef();
void regulator();
void log();
void pidUpdate();

// Serial communication functions
int getPidParamsFromSerial(struct pidParamsTp *pidParams);
int parseLayer2Serial(U8 buf[], U8 maxLen);
int parseLayer3(U8 buf[], struct pidParamsTp *pidParams);

double glob_speedSetpoint;
I32 glob_quadCtr;
struct pidParamsTp glob_PID;

void setup()
{
    TCCR1B = TCCR1B & 0xf8 | 1; // set higher pwm frequency
    EIMSK |= (1 << INT0);  // enable external int
    EICRA |= (1 << ISC01); // trigger INT0 on falling edge
    DDRD = 0xf0; // debug pins and interrupt pin
    DDRB = 0; // baseline: all pins on PORTB are input (true for the quad pins)
    DDRB |= 1<<H_BRIDGE_PWM;
    DDRB |= 1<<H_BRIDGE_DIR;
    glob_quadCtr = 0;
    glob_speedSetpoint = 0;
    glob_PID.P = 0.05;
    glob_PID.I = 4.0;
    glob_PID.D = 10.0;

    Serial.begin(115200);
    while(!Serial);
    Serial.println("SERIAL READY");

    k_init(4, 7, 1);

    // mutex semaphores for protecting shared data
    mutSemSpeedRef  = k_crt_sem(1, 1);
    mutSemQuadCtr   = k_crt_sem(1, 1);
    mutSemPID       = k_crt_sem(1, 1);

    // timer semaphores
    timerSemSpeedRef  = k_crt_sem(0, 1);
    timerSemControl   = k_crt_sem(0, 1);
    timerSemPIDUpdate = k_crt_sem(0, 1);
    timerSemLog       = k_crt_sem(0, 1);

    // message queues
    logMsgQ = k_crt_send_Q(LOG_MSG_Q_LEN, sizeof(struct logMsg), logDataBufForMsgQ);

    // create tasks
    pTaskSpeedRef  = k_crt_task(speedRef,  SPEEDREF_PRIO,  stackSpeedRef,  STACKSIZE);
    pTaskRegulator = k_crt_task(regulator, REGULATOR_PRIO, stackRegulator, STACKSIZE);
    pTaskLog       = k_crt_task(log,       LOG_PRIO,       stackLog,       STACKSIZE);
    pTaskPIDUpdate = k_crt_task(pidUpdate, PIDUPDATE_PRIO, stackPIDUpdate, STACKSIZE);

    k_start();

    // Hopefully unreachable
    Serial.println("FATAL ERROR: krnl failed to start");
}

void loop(){} // never called: just here for compilation

// Read from analog pin
void speedRef()
{
    k_set_sem_timer(timerSemSpeedRef, SPEEDREF_PERIOD);
    while (1) {
        k_wait(timerSemSpeedRef, 0);

        PORTD |= 1<<SPEEDREF_DBG_PIN;

        k_wait(mutSemSpeedRef, SPEEDREF_TIMEOUT);
        // subtract 337 because of sign and using 3.3V ref
        glob_speedSetpoint = (analogRead(SPEEDREF_PIN)-337)*SPEEDREF_COEFF;
        k_signal(mutSemSpeedRef);

        PORTD &= ~(1<<SPEEDREF_DBG_PIN);
    }
}

void regulator()
{
    struct pidParamsTp localPidParams;
    struct logMsg localLogMsg;
    double localSpeedRef;
    double quadSpeed = 0;
    double lastSpeed = 0;
    double error;
    double iTerm;
    double dTerm;
    I32 oldQuadCtr = 0;
    I32 newQuadCtr = 0;
    U32 oldTime = 0;
    U32 newTime = 0;
    U8 dutyCycle = 0;
    U8 direction = 0;
    k_set_sem_timer(timerSemControl, CONTROL_PERIOD);
    while (1) {
        k_wait(timerSemControl, 0);
        PORTD |= 1<<CONTROL_DBG_PIN;

        // get global pid params
        k_wait(mutSemPID, PID_TIMEOUT);
        localPidParams = glob_PID;
        k_signal(mutSemPID);

        // get global speed ref
        k_wait(mutSemSpeedRef, SPEEDREF_TIMEOUT);
        localSpeedRef = glob_speedSetpoint;
        k_signal(mutSemSpeedRef);

        // get quad speed
        oldQuadCtr = newQuadCtr;
        oldTime = newTime;
        k_wait(mutSemQuadCtr, QUADSPEED_TIMEOUT);
        newQuadCtr = glob_quadCtr;
        k_signal(mutSemQuadCtr);
        newTime = millis();
        quadSpeed = (1.0*newQuadCtr - 1.0*oldQuadCtr)/(1.0*newTime - 1.0*oldTime);

        // PID calculation and control
        error = localSpeedRef - quadSpeed;
        // calculate the integral term
        iTerm += localPidParams.I * error * CONTROL_PERIOD;
        // calculate the derivative term
        dTerm = localPidParams.D * (quadSpeed - lastSpeed) / CONTROL_PERIOD;
        lastSpeed = quadSpeed;
        // calculate the final duty cycle
        dutyCycle = localPidParams.P * error + iTerm + dTerm;
        direction = error > 0; // set direction bit based on the sign of the error
        if (dutyCycle > pwmMaxDc) // maximum PWM output safety so the setup doesn't vibrate itself to the brink of destruction
            dutyCycle = pwmMaxDc;
        setBitNToX(PORTB, H_BRIDGE_DIR, direction); // set direction
        analogWrite(H_BRIDGE_PWM_NUMERIC, dutyCycle); // set PWM output

        // Send log messages
#ifndef SUPPRESS_REGULATOR_LOG
        localLogMsg.id = LOG_REGULATOR_ID;
#ifdef LOG_ALL
        localLogMsg.logType = LOG_TIME_PREFIX;
        *(U32 *)localLogMsg.data = newTime;
        *(U32 *)(localLogMsg.data+4) = oldTime;
        *(U32 *)(localLogMsg.data+8) = 0; // clear unused data field
        k_send(logMsgQ, &localLogMsg);
        localLogMsg.logType = LOG_COUNTER_PREFIX;
        *(U32 *)localLogMsg.data = newQuadCtr;
        *(U32 *)(localLogMsg.data+4) = oldQuadCtr;
        *(U32 *)(localLogMsg.data+8) = 0; // clear unused data field
        k_send(logMsgQ, &localLogMsg);
#endif // LOG_ALL
        localLogMsg.logType = LOG_SPEED_PREFIX;
        *(double *)localLogMsg.data = quadSpeed;
        *(double *)(localLogMsg.data+4) = localSpeedRef;
        *(U32 *)(localLogMsg.data+8) = 0; // clear unused data field
        k_send(logMsgQ, &localLogMsg);
#endif // SUPPRESS_REGULATOR_LOG

        PORTD &= ~(1<<CONTROL_DBG_PIN);
    }
}

void log()
{
    struct logMsg localLogMsg;
    k_set_sem_timer(timerSemLog, LOG_PERIOD);
    while (1) {
        k_wait(timerSemLog, 0);
        PORTD |= 1<<LOG_DBG_PIN;

        k_receive(logMsgQ, &localLogMsg, 0, NULL);

        // Send log message properly to the Serial monitor
        Serial.print((char)localLogMsg.id);
        Serial.print(": ");
        Serial.print((char)localLogMsg.logType);
        Serial.print(", ");
        if (localLogMsg.logType == 'S' || localLogMsg.logType == 'P') { // interpret data field as two doubles
            for (int i=0; i<LOGMSG_DATA_LEN; i+=4) {
                Serial.print(*(double *)(localLogMsg.data+i));
                Serial.print(", ");
            }
        }
        else if (localLogMsg.logType == 'C') { // interpret data field as two I32s
            for (int i=0; i<LOGMSG_DATA_LEN; i+=4) {
                Serial.print(*(I32 *)(localLogMsg.data+i));
                Serial.print(", ");
            }
        }
        else { // interpret data field as two U32s
            for (int i=0; i<LOGMSG_DATA_LEN; i+=4) {
                Serial.print(*(U32 *)(localLogMsg.data+i));
                Serial.print(", ");
            }
        }
        Serial.println();

        PORTD &= ~(1<<LOG_DBG_PIN);
    }
}

void pidUpdate()
{
    k_set_sem_timer(timerSemPIDUpdate, PIDUPDATE_PERIOD);
    struct pidParamsTp localPidParams;
    struct logMsg localLogMsg;
    while (1) {
        k_wait(timerSemPIDUpdate, 0);
        PORTD |= 1<<PIDUPDATE_DBG_PIN;

        // read new values from serial monitor
        if (0 <= getPidParamsFromSerial(&localPidParams)) {
            // get semaphore
            k_wait(mutSemPID, 0);
            // set new values for PID object
            glob_PID = localPidParams;
            // release semaphore
            k_signal(mutSemPID);
            // log the new pid params
            localLogMsg.id = LOG_SERIAL_ID;
            localLogMsg.logType = LOG_PID_PREFIX;
            memcpy(localLogMsg.data, &localPidParams, LOGMSG_DATA_LEN);
            k_send(logMsgQ, &localLogMsg);
        }
        PORTD &= ~(1<<PIDUPDATE_DBG_PIN);
    }
}

int getPidParamsFromSerial(struct pidParamsTp *pidParams)
{
    int retVal;
    U8 layer3Buf[MAXLEN_LAYER3];
    retVal = parseLayer2Serial(layer3Buf, MAXLEN_LAYER3);
    if (retVal < 0)
        return retVal;
    retVal = parseLayer3(layer3Buf, pidParams);
    return retVal;
}

// Writes layer 3 message to buf
// caller responsible for allocating large enough buffer
// Return values
// -1: serial monitor empty
// -2: layer 3 buffer overflow prevention (stop condition not received appropriately)
int parseLayer2Serial(U8 buf[], U8 maxLen)
{
    // Return if no new packets are present
    if (!Serial.available())
        return -1;

    U8 index;
    U8 charbuf;
    U8 receiving = 0;
    U8 ignoreNext = 0;
    // read 1 byte
    do {
        // Wait until data is valid in the serial buffer
        // Very suboptimal since it will hold the program until it receives data
        while (!Serial.available());

        Serial.readBytes(&charbuf, 1);
        switch (charbuf) {
            case PROTO_STARTCONDITION:
                if (ignoreNext)
                    goto read_byte;
                receiving = 1;
                index = 0;
                break;
            case PROTO_STOPCONDITION:
                if (ignoreNext)
                    goto read_byte;
                receiving = 0;
                break;
            case PROTO_ESCAPECONDITION:
                if (ignoreNext)
                    goto read_byte;
                ignoreNext = 1;
                break;
            default:
            read_byte:
                if (receiving && index < maxLen) {
                    buf[index] = charbuf;
                    index++;
                }
                else if (index >= maxLen) {
                    return -2; // overflow prevention
                }
                break;
        }
    } while (receiving);
    memset(buf+index, 0, maxLen-index-1);
    return index+1;
}

// only understands PID update packets
// Return values
// -2: Not a PID update packet
// -3: Malformed packet
int parseLayer3(U8 buf[], struct pidParamsTp *pidParams)
{
    char charbuf;
    double newParams[3];
    // read 1 byte and check that it is a PID update packet
    if (*buf++ != PROTO_PIDUPDATE_PREFIX)
        return -2;

    for (int i=0; i<3; i++) {
        newParams[i] = strtod(buf, &buf);
        if (buf == NULL)
            return -3;
        buf++; // increment past the ','
    }

    pidParams->P = newParams[0];
    pidParams->I = newParams[1];
    pidParams->D = newParams[2];
    return 0;
}

// Increment glob_quadCtr ISR
ISR(INT0_vect, ISR_NAKED)
{
    // no local vars
    PUSHREGS();
    if (!k_running)
        goto exit_ISR;

    if (PINB & (1<<PIND_QUAD_SIGN_PIN)) // check if clockwise rotation
        glob_quadCtr--;
    else
        glob_quadCtr++;

    K_CHG_STAK();
exit_ISR:
    POPREGS();
    RETI();
}
