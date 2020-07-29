package com.example.mobile_controller;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.*;//소켓 프로그래밍 관련 패키지
import android.Manifest;
import android.app.AlertDialog;
import android.content.DialogInterface;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.content.Intent;//음성인식 관련 패키지
import android.os.Bundle;
import android.support.v4.app.ActivityCompat;//오디오녹음 권한 허용창 생성에 필요
import android.support.v4.content.ContextCompat;//오디오녹음 권한 허용창 생성에 필요
import android.support.v7.app.AppCompatActivity;
import android.speech.RecognizerIntent;//음성인식 관련 패키지
import android.speech.RecognitionListener;//음성인식 관련 패키지
import android.speech.SpeechRecognizer;//음성인식 관련 패키지
import android.text.Editable;
import android.text.TextWatcher;
import android.view.MotionEvent;
import android.view.View;
import android.widget.CompoundButton;
import android.widget.ImageButton;
import android.widget.Switch;
import android.widget.TextView;
import android.widget.Toast;

import java.util.ArrayList;
import java.util.Locale;

public class MainActivity extends AppCompatActivity {

    View dialogView;
    String tmp;
    Integer speed;

    Socket socket = null;//소켓 변수
    InputStream inputStream = null;//입력스트림 변수
    OutputStream outputStream  = null;//출력스트림 변수
    byte[] bufSnd = new byte[1];

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

        //SharedPreference에서 속도 값을 가져와서 속도계에 출력
        SharedPreferences sp = getSharedPreferences("SPEED", MODE_PRIVATE);
        speed = sp.getInt("speed", 120);
        speedView.setText(speed.toString());

        try {
            socket = new Socket();
            SocketAddress addr = new InetSocketAddress("192.168.219.104", 9000);
            socket.connect(addr);
            outputStream = socket.getOutputStream();

        } catch (IOException e) {
            e.printStackTrace();
            Toast.makeText(MainActivity.this, e.toString(), Toast.LENGTH_SHORT).show();
        } catch (SecurityException se) {
            // security manager에서 허용되지 않은 기능 수행.
            Toast.makeText(MainActivity.this, se.toString(), Toast.LENGTH_SHORT).show();
        } catch (IllegalArgumentException ae) {
            // 소켓 생성 시 전달되는 포트 번호(65536)이 허용 범위(0~65535)를 벗어남.
            Toast.makeText(MainActivity.this, ae.toString(), Toast.LENGTH_SHORT).show();
        }

        //전진 버튼 리스너 -> 이미지변경
        go.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction())
                {
                    case MotionEvent.ACTION_DOWN:
                        go.setBackgroundResource(R.drawable.up_pushed);

                        bufSnd[0] = 0x00;
                        try {
                            outputStream.write(bufSnd, 0, 1);
                        } catch (IOException e) {
                            e.printStackTrace();
                        }
                        break;

                    case MotionEvent.ACTION_UP:
                        go.setBackgroundResource(R.drawable.up_button);
                        break;
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
                        back.setBackgroundResource(R.drawable.down_pushed);

                        break;

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
                        speedUp.setBackgroundResource(R.drawable.speed_up_button);
                        tmp = speedView.getText().toString();
                        speed = Integer.parseInt(tmp);
                        if(speed >= 255) speed = 255;
                        else             speed += 5;
                        speedView.setText(speed.toString());
                        break;
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
                        speedDown.setBackgroundResource(R.drawable.speed_down_button);
                        tmp = speedView.getText().toString();
                        speed = Integer.parseInt(tmp);
                        if(speed <= 0) speed = 0;
                        else           speed -= 5;
                        speedView.setText(speed.toString());
                        break;
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

                    Toast.makeText(MainActivity.this, "음성인식을 시작합니다.", Toast.LENGTH_SHORT).show();

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

                    inputVoice(voice_text);
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

                    Toast.makeText(MainActivity.this, "음성인식을 종료합니다.", Toast.LENGTH_SHORT).show();
                }
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

    //음성인식 처리부
    public void inputVoice(final TextView voice_text) {
        try {
            Intent intent = new Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH);
            intent.putExtra(RecognizerIntent.EXTRA_CALLING_PACKAGE, this.getPackageName());
            intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE, "ko-KR");
            final SpeechRecognizer stt = SpeechRecognizer.createSpeechRecognizer(this);
            stt.setRecognitionListener(new RecognitionListener() {
                @Override
                public void onReadyForSpeech(Bundle params) {
                    Toast.makeText(MainActivity.this, "명령해주십시오", Toast.LENGTH_SHORT).show();
                }

                @Override
                public void onBeginningOfSpeech() {

                }

                @Override
                public void onRmsChanged(float rmsdB) {

                }

                @Override
                public void onBufferReceived(byte[] buffer) {

                }

                @Override
                public void onEndOfSpeech() {
                    Toast.makeText(MainActivity.this, "명령 인식 완료", Toast.LENGTH_SHORT).show();
                }

                @Override
                public void onError(int error) {
                    Toast.makeText(MainActivity.this, "오류 발생 : " + error, Toast.LENGTH_SHORT).show();
                }

                @Override
                public void onResults(Bundle results) {
                    ArrayList<String> result = (ArrayList<String>) results.get(SpeechRecognizer.RESULTS_RECOGNITION);
                    voice_text.setText("");
                    voice_text.append("[명령] " + result.get(0) + "\n");
                    replyAnswer(result.get(0), voice_text);
                    stt.destroy();
                }

                @Override
                public void onPartialResults(Bundle partialResults) {

                }

                @Override
                public void onEvent(int eventType, Bundle params) {

                }
            });
        } catch(Exception e) {
            Toast.makeText(MainActivity.this, e.toString(), Toast.LENGTH_LONG).show();
        }
    }

    //인식된 음성에 따른 출력 결과 처리부
    public void replyAnswer(String input, TextView voice_text) {
        try {
            if(input.equals("앞으로 가")) {
                voice_text.append(">> 차량을 앞으로 움직입니다.");
            }

            else if(input.equals("뒤로 가")) {
                voice_text.append(">> 차량을 뒤로 움직입니다.");
            }

            else if(input.contains("왼쪽 차선") && input.contains("이동")) {
                voice_text.append(">> 왼쪽 차선으로 이동합니다.");
            }

            else if(input.contains("오른쪽 차선") && input.contains("이동")) {
                voice_text.append(">> 오른쪽 차선으로 이동합니다.");
            }

            else if(input.contains("왼쪽") && input.contains("전진")) {
                voice_text.append(">> 왼쪽으로 전진합니다.");
            }

            else if(input.contains("오른쪽") && input.contains("전진")) {
                voice_text.append(">> 오른쪽 으로 전진합니다.");
            }

            else if(input.contains("왼쪽") && input.contains("후진")) {
                voice_text.append(">> 왼쪽으로 후진합니다.");
            }

            else if(input.contains("오른쪽") && input.contains("후진")) {
                voice_text.append(">> 오른쪽으로 후진합니다.");
            }

            else if(input.equals("속도 올려") || input.equals("속도 높여")) {
                voice_text.append(">> 차량의 속도를 높입니다.");

                final TextView speedView = findViewById(R.id.speedView);
                if(speed >= 255) speed = 255;
                else             speed += 5;
                speedView.setText(speed.toString());
            }

            else if(input.equals("속도 내려") || input.equals("속도 낮춰")) {
                voice_text.append(">> 차량의 속도를 내립니다.");

                final TextView speedView = findViewById(R.id.speedView);
                if(speed <= 0) speed = 0;
                else             speed -= 5;
                speedView.setText(speed.toString());
            }

            else if(input.equals("정지")) {
                voice_text.append(">> 차량을 정지합니다.");
            }

            else if(input.equals("종료")) {
                voice_text.append(">> 음성인식 기능이 종료단계에 들어갑니다....");
                finish();
            }

            else {
                voice_text.append("** 알 수 없는 명령입니다. **");
            }

        } catch(Exception e) {
            Toast.makeText(MainActivity.this, e.toString(), Toast.LENGTH_SHORT).show();
        }
    }

    protected void onDestroy() {
        super.onDestroy();
        /*
        try {
            if(outputStream != null)
                socket.close();
        } catch (IOException e) {
            e.printStackTrace();
            Toast.makeText(MainActivity.this, e.toString(), Toast.LENGTH_SHORT).show();
        } catch (SecurityException se) {
          // security manager에서 허용되지 않은 기능 수행.
            Toast.makeText(MainActivity.this, se.toString(), Toast.LENGTH_SHORT).show();
        } catch (IllegalArgumentException ae) {
            // 소켓 생성 시 전달되는 포트 번호(65536)이 허용 범위(0~65535)를 벗어남.
            Toast.makeText(MainActivity.this, ae.toString(), Toast.LENGTH_SHORT).show();
        }
        */
    }
}