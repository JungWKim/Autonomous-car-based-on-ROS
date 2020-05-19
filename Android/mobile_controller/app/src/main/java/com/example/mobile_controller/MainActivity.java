package com.example.mobile_controller;

import android.Manifest;
import android.app.AlertDialog;
import android.content.DialogInterface;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.os.Bundle;
import android.support.v4.app.ActivityCompat;//오디오녹음 권한 허용창 생성에 필요
import android.support.v4.content.ContextCompat;//오디오녹음 권한 허용창 생성에 필요
import android.support.v7.app.AppCompatActivity;
import android.view.MotionEvent;
import android.view.View;
import android.widget.CompoundButton;
import android.widget.ImageButton;
import android.widget.Switch;
import android.widget.TextView;
import android.widget.Toast;

public class MainActivity extends AppCompatActivity {

    View dialogView;
    String tmp;
    Integer speed;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        if(ContextCompat.checkSelfPermission(this, Manifest.permission.RECORD_AUDIO) != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(this, new String[] {Manifest.permission.RECORD_AUDIO}, 5);
            Toast.makeText(MainActivity.this, "허용하지 않을 경우 음성인식기능에 제한이 발생할 수 있습니다.", Toast.LENGTH_LONG).show();
        }

        final ImageButton go = findViewById(R.id.go);
        final ImageButton back = findViewById(R.id.back);
        final ImageButton left = findViewById(R.id.left);
        final ImageButton right = findViewById(R.id.right);
        final ImageButton speedUp = findViewById(R.id.speedUp);
        final ImageButton speedDown = findViewById(R.id.speedDown);

        final TextView speedView = findViewById(R.id.speedView);
        final TextView voice_text = findViewById(R.id.voice_text);

        final Switch autoDriveMode = findViewById(R.id.autoDriveMode);
        final Switch voiceControl = findViewById(R.id.voiceControl);

        SharedPreferences sp = getSharedPreferences("SPEED", MODE_PRIVATE);
        speed = sp.getInt("speed", 120);
        speedView.setText(speed.toString());

        //전진 버튼 리스너 -> 이미지변경
        go.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction())
                {
                    case MotionEvent.ACTION_DOWN:
                        go.setBackgroundResource(R.drawable.up_pushed); break;

                    case MotionEvent.ACTION_UP:
                        go.setBackgroundResource(R.drawable.up_button); break;
                }

                return false;
            }
        });

        //후진 버튼 리스너 -> 이미지 변경
        back.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction())
                {
                    case MotionEvent.ACTION_DOWN:
                        back.setBackgroundResource(R.drawable.down_pushed); break;

                    case MotionEvent.ACTION_UP:
                        back.setBackgroundResource(R.drawable.down_button); break;
                }

                return false;
            }
        });

        //좌회전 버튼 리스너 -> 이미지 변경
        left.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction())
                {
                    case MotionEvent.ACTION_DOWN:
                        left.setBackgroundResource(R.drawable.left_pushed); break;

                    case MotionEvent.ACTION_UP:
                        left.setBackgroundResource(R.drawable.left_button); break;
                }

                return false;
            }
        });

        //우회전 버튼 리스너 -> 이미지 변경
        right.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction())
                {
                    case MotionEvent.ACTION_DOWN:
                        right.setBackgroundResource(R.drawable.right_pushed); break;

                    case MotionEvent.ACTION_UP:
                        right.setBackgroundResource(R.drawable.right_button); break;
                }

                return false;
            }
        });

        //속도 증가 버튼 리스너 -> 이미지 변경
        speedUp.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction())
                {
                    case MotionEvent.ACTION_DOWN:
                        speedUp.setBackgroundResource(R.drawable.speed_up_pushed); break;

                    case MotionEvent.ACTION_UP:
                        speedUp.setBackgroundResource(R.drawable.speed_up_button); break;
                }

                return false;
            }
        });

        //속도 감소 버튼 -> 이미지 변경
        speedDown.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction())
                {
                    case MotionEvent.ACTION_DOWN:
                        speedDown.setBackgroundResource(R.drawable.speed_down_pushed); break;

                    case MotionEvent.ACTION_UP:
                        speedDown.setBackgroundResource(R.drawable.speed_down_button); break;
                }

                return false;
            }
        });

        //음성인식 스위치 리스너
        voiceControl.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {

                //음성인식 스위치가 켜졌을 때 -> 나머지 버튼 비활성화 및 이미지 변경
                if(isChecked) {
                    go.setEnabled(false);
                    back.setEnabled(false);
                    speedUp.setEnabled(false);
                    speedDown.setEnabled(false);
                    left.setEnabled(false);
                    right.setEnabled(false);

                    go.setBackgroundResource(R.drawable.up_locked);
                    back.setBackgroundResource(R.drawable.down_locked);
                    speedUp.setBackgroundResource(R.drawable.speed_up_locked);
                    speedDown.setBackgroundResource(R.drawable.speed_down_locked);
                    left.setBackgroundResource(R.drawable.left_locked);
                    right.setBackgroundResource(R.drawable.right_locked);

                    Toast.makeText(MainActivity.this, "음성인식을 시작합니다.", Toast.LENGTH_LONG).show();

                    //음성인식 스위치 켜질 경우 -> 사용자가 한 말을 보여주는 커스텀다이얼로그 생성
                    dialogView = getLayoutInflater().inflate(R.layout.voice_dialog, null);//레이아웃을 담는 View객체 생성
                    AlertDialog.Builder builder = new AlertDialog.Builder(MainActivity.this);//alertDialog.builder객체 생성 -> 다이얼로그의 각종 setting가능
                    builder.setView(dialogView);
                    builder.setTitle("Voice Controlling");//다이얼로그 타이틀
                    builder.setCancelable(false);//다이얼로그 외부 눌러도 종료되지 않게 함
                    //다이얼로그의 종료 버튼 리스너
                    builder.setPositiveButton("종료", new DialogInterface.OnClickListener() {
                        @Override
                        public void onClick(DialogInterface dialog, int which) {
                            voiceControl.setChecked(false);
                        }
                    });
                    AlertDialog alertDialog = builder.create();//AlertDialog객체 생성
                    alertDialog.show();
                }

                //음성인식 스위치가 꺼졌을 때 -> 나머지 버튼 비활성화 및 이미지 변경
                else {
                    go.setEnabled(true);
                    back.setEnabled(true);
                    speedUp.setEnabled(true);
                    speedDown.setEnabled(true);
                    left.setEnabled(true);
                    right.setEnabled(true);

                    go.setBackgroundResource(R.drawable.up_button);
                    back.setBackgroundResource(R.drawable.down_button);
                    speedUp.setBackgroundResource(R.drawable.speed_up_button);
                    speedDown.setBackgroundResource(R.drawable.speed_down_button);
                    left.setBackgroundResource(R.drawable.left_button);
                    right.setBackgroundResource(R.drawable.right_button);

                    Toast.makeText(MainActivity.this, "음성인식을 종료합니다.", Toast.LENGTH_LONG).show();
                }
            }
        });

        //속도 증가 버튼 리스너 -> 속도 5씩 높이고 텍스트뷰 표시
        speedUp.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                tmp = speedView.getText().toString();
                speed = Integer.parseInt(tmp);
                if(speed >= 255) speed = 255;
                else speed += 5;
                speedView.setText(speed.toString());
            }
        });

        //속도 감소 버튼 리스너 -> 속도 5씩 줄이고 텍스트뷰 표시
        speedDown.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                tmp = speedView.getText().toString();
                speed = Integer.parseInt(tmp);
                if(speed <= 0) speed = 0;
                else speed -= 5;
                speedView.setText(speed.toString());
            }
        });

        //자율주행 스위치 리스너
        autoDriveMode.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {

                //자율주행 스위치가 켜졌을 때 -> 나머지 버튼 비활성화 및 이미지 변경
                if(isChecked) {
                    go.setEnabled(false);
                    back.setEnabled(false);
                    speedUp.setEnabled(false);
                    speedDown.setEnabled(false);
                    left.setEnabled(false);
                    right.setEnabled(false);

                    go.setBackgroundResource(R.drawable.up_locked);
                    back.setBackgroundResource(R.drawable.down_locked);
                    speedUp.setBackgroundResource(R.drawable.speed_up_locked);
                    speedDown.setBackgroundResource(R.drawable.speed_down_locked);
                    left.setBackgroundResource(R.drawable.left_locked);
                    right.setBackgroundResource(R.drawable.right_locked);

                    Toast.makeText(MainActivity.this, "자율주행을 시작합니다.", Toast.LENGTH_LONG).show();
                }

                //자율주행 스위치가 꺼졌을 때 -> 나머지 버튼 비활성화 및 이미지 변경
                else {
                    go.setEnabled(true);
                    back.setEnabled(true);
                    speedUp.setEnabled(true);
                    speedDown.setEnabled(true);
                    left.setEnabled(true);
                    right.setEnabled(true);

                    go.setBackgroundResource(R.drawable.up_button);
                    back.setBackgroundResource(R.drawable.down_button);
                    speedUp.setBackgroundResource(R.drawable.speed_up_button);
                    speedDown.setBackgroundResource(R.drawable.speed_down_button);
                    left.setBackgroundResource(R.drawable.left_button);
                    right.setBackgroundResource(R.drawable.right_button);

                    Toast.makeText(MainActivity.this, "자율주행을 종료합니다.", Toast.LENGTH_LONG).show();
                }
            }
        });
    }

    @Override
    protected void onStop()
    {
        super.onStop();

        final TextView speedView = findViewById(R.id.speedView);

        SharedPreferences sp = getSharedPreferences("SPEED", MODE_PRIVATE);
        SharedPreferences.Editor editor = sp.edit();
        String tmp = speedView.getText().toString();
        editor.putInt("speed", Integer.parseInt(tmp));

        editor.commit();
    }
}