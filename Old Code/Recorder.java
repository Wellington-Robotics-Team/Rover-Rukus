package org.firstinspires.ftc.robotcontroller.internal;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.StreamCorruptedException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by David on 11/9/2017.
 */

public class Recorder {
    //file needed for logging
    private File LogFile;
    private double oldRuntime; //the last runtime, used to determine when to save things to list
    private Object[] dataset;//used to record, contains runtime and 2 gamepads
    private ArrayList<Object[]> alldatasets;//list used to hold datasets
    private double definition; //used to record and play back - how much space there is between each dataset (seconds)
    //used to play back - gamepads set after being read out of a file
    private Gamepad currentgp1;
    private Gamepad currentgp2;
    //used to record, set to gamepad 1 and 2 in opmode and then saved
    private Gamepad gamepad1;
    private Gamepad gamepad2;
    //used to play back - key is the time and object[] is an array with g1 as 0 and g2 as 1
    private HashMap<Double,Object[]> hmap;

    public Recorder(double def, String title){
        //for playback
        oldRuntime = 0;
        definition = def;
        alldatasets = new ArrayList<Object[]>();
        dataset = new Object[3];//runtime, gamepad1, gamepad2
        LogFile = new File("/storage/emulated/0/"+title+".ser");//this is the path that will be saved to
        //used to use testrecord.ser,record.ser

        //initialize so when playing back it doesn't try to use properties of a null gamepad
        currentgp1 = new Gamepad();
        currentgp2 = new Gamepad();
    }
    public Recorder(double def){
        //for playback
        oldRuntime = 0;
        definition = def;
        alldatasets = new ArrayList<Object[]>();
        dataset = new Object[3];//runtime, gamepad1, gamepad2
        LogFile = new File("/storage/emulated/0/record.ser");//this is the path that will be saved to
        //used to use testrecord.ser,record.ser

        //initialize so when playing back it doesn't try to use properties of a null gamepad
        currentgp1 = new Gamepad();
        currentgp2 = new Gamepad();
    }
    public Recorder(double def, String title, Gamepad gamepad1, Gamepad gamepad2){
        //for recording
        oldRuntime = 0;
        //sets makes the objects' gamepads the opmodes gamepads - this way i don't have to get them every loop.
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        definition = def;
        alldatasets = new ArrayList<Object[]>();
        dataset = new Object[3];//see other constructor
        LogFile = new File("/storage/emulated/0/"+title+".ser");//this is the path that will be saved to
        //used to use testrecord.ser,record.ser
    }
    public Recorder(double def, Gamepad gamepad1, Gamepad gamepad2){
        //for recording
        oldRuntime = 0;
        //sets makes the objects' gamepads the opmodes gamepads - this way i don't have to get them every loop.
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        definition = def;
        alldatasets = new ArrayList<Object[]>();
        dataset = new Object[3];//see other constructor
        LogFile = new File("/storage/emulated/0/record.ser");//this is the path that will be saved to
        //used to use testrecord.ser,record.ser
    }
    public void record(double runtime, Telemetry telemetry){
        //only record if the time has run forward
        if(runtime>= oldRuntime+definition)
        {
            //re-instantiate every time to avoid a weird memory thing
            dataset = new Object[3];
            //put the time, rounded to the tenths in the dataset array (of objects)
            dataset[0] = (double)Math.round(runtime*10)/10;

            try {
                //set the gamepads to a byte array which is serializable- gamepads aren't.
                dataset[1] = gamepad1.toByteArray();//we should always have 1 controller connected
                if(gamepad2!= null) dataset[2] = gamepad2.toByteArray();//this might cause issues on the read end.
            } catch (RobotCoreException e) {
                //catches error (idk what causes this)
                e.printStackTrace();
                telemetry.addData("error","to byte array error!!");
                telemetry.update();
                try {
                    Thread.sleep(1000);//so you can read error in telemetry
                } catch (InterruptedException e1) {
                    e1.printStackTrace();
                }
            }


            //telemetry.addData("Arr: ",dataset[0]);//doesn't work great
            //add to datasets array and set a new time to check the change in time against
            alldatasets.add(dataset);
            oldRuntime = runtime;
        }
        if (gamepad1.start) {
            //actually writes to the thing
            while(gamepad1.start) {
                telemetry.addData("Working", "Unsaved");//just a heads up that it's working, kills time
                telemetry.update();
            }
            try(
                    //this is like a using statement, but I close them anyway
                    FileOutputStream fos = new FileOutputStream(LogFile);
                    ObjectOutputStream oos = new ObjectOutputStream(fos);
            )
            {
                //writes the list of everything
                oos.writeObject(alldatasets);
                oos.close();
                fos.close();
                telemetry.addData("Working","Saved");
                telemetry.update();
            }
            catch (FileNotFoundException e) {
                e.printStackTrace();
                telemetry.addData("File:","!!!Not Found!!!");
                telemetry.update();
                try {
                    Thread.sleep(1000);//important error to see
                } catch (InterruptedException e1) {
                    e1.printStackTrace();
                }
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }


    public HashMap<Double,Object[]> getData(){
        //this gets all of the logs from the last saved file you are pointing to.
        //should be run in init when you are playing back.

        //instantiate all the stuffs we need later
        List<Object[]> alist = new ArrayList<Object[]>();//just a list to copy to; android studio is unhappy otherwise
        HashMap<Double,Object[]> htable = new HashMap<Double,Object[]>();//used to be a hashtable, holds all data read

        try(
                FileInputStream fis = new FileInputStream(LogFile);
                ObjectInputStream ois = new ObjectInputStream(fis);
        ){
            List<Object[]> list = (List<Object[]>)ois.readObject();
            alist = list;//android studio weirdness
            for (Object[] oneset:alist){
                dataset = new Object[2];
                //all the news make new memory spaces - very important
                Gamepad g1 = new Gamepad();
                g1.fromByteArray((byte[])oneset[1]);
                Gamepad g2 = new Gamepad();
                g2.fromByteArray((byte[])oneset[2]);
                dataset[0] = g1;
                dataset[1] = g2;
                htable.put((double)oneset[0], dataset);//puts into hmap/table

            }
        }
        catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (StreamCorruptedException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        } catch (ClassNotFoundException e) {
            e.printStackTrace();
        } catch (RobotCoreException e) {
            e.printStackTrace();
        }
        this.hmap = htable;
        return htable;//don't necessarily need to use this, especially because you don't want to run this every loop.
    }

    public void setGamepads(double runtime,HashMap<Double,Object[]> hmap){
        //sets with an hmap

        Double rt = new Double((double)Math.round(runtime*10)/10);
        if (hmap.get(rt) != null) {
            currentgp1 = (Gamepad) hmap.get(rt)[0];
            currentgp2 = (Gamepad) hmap.get(rt)[1];
        }
        else {
            System.out.println("error: hmap doesn't have a gamepad there");
        }
    }

    public void setGamepads(double runtime){
        //sets using an hmap already made using getData and this object
        //run every loop, sets gamepads for playback

        Double rt = new Double((double)Math.round(runtime*10)/10);
        if (this.hmap.get(rt) != null) {
            currentgp1 = (Gamepad) hmap.get(rt)[0];
            currentgp2 = (Gamepad) hmap.get(rt)[1];
        }
        else {
            System.out.println("error: hmap doesn't have a gamepad there");
        }
    }

    public Gamepad getG1() {
        return currentgp1;
    }

    public Gamepad getG2() {
        return currentgp2;
    }

}
