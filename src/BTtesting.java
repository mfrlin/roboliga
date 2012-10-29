import java.io.*;
import javax.bluetooth.*;
import lejos.nxt.*;
import lejos.nxt.comm.*;
public class BTtesting {

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		RemoteDevice btrd = Bluetooth.getKnownDevice("ROBI");
		if(btrd == null){
			System.exit(1);
		}
		BTConnection btc = Bluetooth.connect(btrd);
		if(btc == null){
			System.exit(1);
		}
		
		DataInputStream dis = btc.openDataInputStream();
	    DataOutputStream dos = btc.openDataOutputStream();
	    
	    
	}

}
