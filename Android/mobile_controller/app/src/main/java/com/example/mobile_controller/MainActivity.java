package com.example.mobile_controller;

import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.view.View;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.Switch;
import android.widget.TextView;

public class MainActivity extends AppCompatActivity {

    String tmp;
    Integer speed;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        final Button go = findViewById(R.id.go);
        final Button back = findViewById(R.id.back);
        final Button left = findViewById(R.id.left);
        final Button right = findViewById(R.id.right);
        final Button speedUp = findViewById(R.id.speedUp);
        final Button speedDown = findViewById(R.id.speedDown);

        final TextView speedView = findViewById(R.id.speedView);

        Switch autoDriveMode = findViewById(R.id.autoDriveMode);
        Switch voiceControl = findViewById(R.id.voiceControl);

        //음성인식 스위치 리스너
        voiceControl.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {

                if(isChecked) {
                    go.setActivated(false);
                    back.setActivated(false);
                    speedUp.setActivated(false);
                    speedDown.setActivated(false);
                    left.setActivated(false);
                    right.setActivated(false);
                }

                else {
                    go.setActivated(true);
                    back.setActivated(true);
                    speedUp.setActivated(true);
                    speedDown.setActivated(true);
                    left.setActivated(true);
                    right.setActivated(true);
                }
            }
        });

        //속도 증가 리스너
        speedUp.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                tmp = speedView.getText().toString();
                speed = Integer.parseInt(tmp);
                if(speed >= 255) speed = 255;
                else speed++;
                speedView.setText(speed.toString());
            }
        });

        //속도 감소 리스너
        speedDown.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                tmp = speedView.getText().toString();
                speed = Integer.parseInt(tmp);
                if(speed <= 0) speed = 0;
                else speed--;
                speedView.setText(speed.toString());
            }
        });

        //자율주행 스위치 리스너
        autoDriveMode.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {

                if(isChecked) {
                    go.setActivated(false);
                    back.setActivated(false);
                    speedUp.setActivated(false);
                    speedDown.setActivated(false);
                    left.setActivated(false);
                    right.setActivated(false);
                }

                else {
                    go.setActivated(true);
                    back.setActivated(true);
                    speedUp.setActivated(true);
                    speedDown.setActivated(true);
                    left.setActivated(true);
                    right.setActivated(true);
                }
            }
        });
    }
}
//앱 시작시, 연결 설정
//앱 종료시, 연결 해제
//화면 회전시 또는 다른 앱을 사용하다 돌아왔을때 설정값들이 그대로인지 확인
//어플 실행시, 자율주행차의 속도를 통신으로 받아와서 텍스트뷰에 출력