import processing.serial.*;
import java.io.*;
import java.util.*;

Serial myPort;
String[] subtext;
int[] pt1 = new int[3];
int[] vector = new int[3];
int k = 0;

List<double[]> vector_list = new ArrayList<double[]>();

/* file output formatted like: 
 //time=31(1) Frame(2) raw=1:(3) markers=2:(4) 0)(5) 276.957(6),1481.72(7),747.392(8) 1)(9) -3.63931(10),1448.92(11),388.693(12)
 */

int pos = 0;
int data_length = 12;

void setup() {
  myPort = new Serial(this, "COM10", 57600);
  myPort.bufferUntil('\n');
}

void draw() {
  readData("C:/Users/osnho/Documents/dankrobotarm/arm_test.txt");
}

void readData(String myFileName) {

  File file = new File(myFileName);
  BufferedReader br = null;

  try {
    br = new BufferedReader(new FileReader(file));

    String text=null;

    /* keep reading each line until you get to the end of the file */
    while ((text=br.readLine())!=null) {
      delay(3);// give time for computer to process data

      k+=2; // counter

      subtext = splitTokens(text);

      if (k % 30 == 0) {
        String[] data = split(subtext[1], ',');
        for (int i = 0; i < pt1.length; i++) {
          pt1[i] = int(data[i]);
        }
      }

      if (k % 32 == 0) {
        String[] data = split(subtext[1], ',');
        for (int i = 0; i < pt1.length; i++) {
          vector[i] = pt1[i] - int(data[i]);
          println(vector[i]);
          
          myPort.write(vector[i]);
        }
      }

      if (k == 32) {
        k = 0;
      }
    }
    // repeat!
  }

  catch(FileNotFoundException e) {
    e.printStackTrace();
  }

  catch(IOException e) {
    e.printStackTrace();
  }
}
