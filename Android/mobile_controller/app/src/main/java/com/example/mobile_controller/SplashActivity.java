package com.example.mobile_controller;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;

public class SplashActivity extends Activity {
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        try {
            Thread.sleep(4000); //대기 초 설정
        } catch (Exception e) {
            e.printStackTrace();
        }
        startActivity(new Intent(this, SplashActivity.class));
        finish();
    }
}
