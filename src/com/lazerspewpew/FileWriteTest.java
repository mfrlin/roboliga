package com.lazerspewpew;

import java.io.*;
import lejos.nxt.*;

public class FileWriteTest {
  public void abc() { 
    FileOutputStream out = null; // declare outside the try block
    File data = new File("log.dat");

    try {
      out = new FileOutputStream(data);
    } catch(IOException e) {
    	System.err.println("Failed to create output stream");
    	Button.waitForAnyPress();
    	System.exit(1);
    }

    DataOutputStream dataOut = new DataOutputStream(out);

    float x = 1f;
    int length = 8;

    try { // write
      for(int i = 0 ; i<length; i++ ) {
        dataOut.writeFloat(x);
        x = x*-2.2f; 
      }
      out.close(); // flush the buffer and write the file
    } catch (IOException e) {
      System.err.println("Failed to write to output stream");
    }
    Sound.beep();
    Button.waitForAnyPress();
  }
}