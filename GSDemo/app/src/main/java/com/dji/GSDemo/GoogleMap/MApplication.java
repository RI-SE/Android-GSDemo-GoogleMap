package com.dji.GSDemo.GoogleMap;

import android.app.Application;
import android.content.Context;
import android.os.AsyncTask;
import android.util.Log;

import com.secneo.sdk.Helper;

public class MApplication extends Application {
    static {
        try {
            System.loadLibrary("TCPUDPSocket");
            System.loadLibrary("isoObject_wrap");
        }catch(Exception e)
        {
            Log.wtf("Error", e);
        }

        Task droneTask = new Task();
        droneTask.run();


    }


    private DJIDemoApplication fpvDemoApplication;
    @Override
    protected void attachBaseContext(Context paramContext) {
        super.attachBaseContext(paramContext);
        Helper.install(MApplication.this);
        if (fpvDemoApplication == null) {
            fpvDemoApplication = new DJIDemoApplication();
            fpvDemoApplication.setContext(this);
        }
    }





    @Override
    public void onCreate() {

        super.onCreate();


        //fpvDemoApplication.onCreate();

    }
}




class Task implements Runnable {
    @Override
    public void run() {
        IsoDrone drone = new IsoDrone("192.168.75.127");

        while(true) {
            try {
                Thread.sleep(100);
                Log.wtf("Name", drone.getName());
                Log.wtf("State", drone.getCurrentStateName());
                Log.wtf("IPv4", Utils.getIPAddress(true)); // IPv4

            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

    }
}

/*
class AsyncTaskRunner extends AsyncTask<String, String, String> {
    @Override
    protected String doInBackground(String... params) {
        IsoDrone drone = new IsoDrone("192.168.21.87");
        Log.wtf("IPv4", Utils.getIPAddress(true)); // IPv4
        return "";
    }
    @Override
    protected void onPostExecute(String result) {
    }
    @Override
    protected void onPreExecute() {
    }
    @Override
    protected void onProgressUpdate(String... text) {
    }
}
*/


