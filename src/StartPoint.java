import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.SensorPort;
import lejos.nxt.NXTMotor;
import lejos.nxt.MotorPort;
import lejos.nxt.Motor;

public class StartPoint {

	public static void main(String[] args) {
		NXTMotor levi = new NXTMotor(MotorPort.A); // levi motor je unregulated. poganjamo ga z nastavljanjem voltaze
		Motor.B.setSpeed(360); // nastavimo desni motor na 360 stopinj na sekundo
		PID myPID = new PID(3, 0.05, 0.25); // neke random cifre za testiranje PIDa
		Motor.B.forward();
		// TODO: testirati kakšno natanènost dobimo s PIDom, ki je ze vgrajen v leJOS (lejos.utils)
		while (levi.getTachoCount() < 1440 || Motor.B.getTachoCount() < 1440) {
			// pidu podamo stopinje levega kolesa (input) in stopinje desnega kolesa (setpoint) koliko bi moral biti tudi levi
			int voltaza = (int)myPID.compute(levi.getTachoCount(), Motor.B.getTachoCount()); // pid nam izracuna voltazo
			if (voltaza > -1) { // glede na to, da pid ne vrne nicesar, ce se ni preteklo 5 ms sem to resil, da vracam -1 kar je verjetno hudo narobe
				levi.setPower(voltaza);
			}	
		}
		levi.stop();
		Motor.B.stop();
		LCD.drawInt(levi.getTachoCount(), 0, 0);
		LCD.drawInt(Motor.B.getTachoCount(), 0, 1); // levo kolo je +-10 stopinj razlike od desnega pri mojih testih
		Button.waitForAnyPress();
		}

}
