////////////////////////////////////////////////////////////////////////////////
// Command
////////////////////////////////////////////////////////////////////////////////
#include "command.h"
#include <Arduino.h>

/*
 * There are two kinds of message, Arduino to Commander, and Commander to
 * Arduino. Both have the same format. The distinction is that C2A
 * messages relay the state we wish the arduino to be in, 
 * 
 * Message Format:
 * "BEGIN <MoveCmdID> <MoveCmd> <MoveCmdDir> <MoveCmdSpd> <KickCmdId> <KickCmd> <CatchCmdId> <CatchCmd> <CurrentAngle> END\r\n"
 * Where:
 *  <...CmdID>   = id of command being sent. Non-negative integer.
 *  <...Cmd>     = command. Only the uppercase first letter is sent.
 *  <MoveCmdDir> = direction argument (angle in radians, between -PI and PI). Float.
 *  <MoveCmdSpd> = speed argument. Scale from 0 to 100. Integer.
 *  <MoveCmd>    = One of M for Move, T for Turn, or S for Stop.
 *  <KickCmd>    = One of K for Kick, or I for idle.
 *  <CatchCmd>   = One of C for Catch, or R for Release.
 *  <CurrentAngle> = Current angle we believe the robot is facing.
 */

void setup_command(void)
{
    // TODO
}

void service_command(void)
{
    // TODO
}