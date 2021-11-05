#include "mbed.h"
#include "2021roboSeparateBoardLib_setting.hpp"

//Serial pc(USBTX,USBRX);
//DigitalOut led(LED1);

#ifdef ENABLE_xbeeCore
//xbeeで何かを受信するときの例です
const int RECEIVE_ARRAY_LEN=255;
uint8_t receiveArray[RECEIVE_ARRAY_LEN]={};
uint16_t receiveLen=0;
void whenReceiveFunc(uint8_t *array,uint16_t size){
	//何かを受信したときにすることを書きます
	//ここではすべての値をグローバル変数にコピーしています
	for(int i=0;i<size;i++){
		receiveArray[i]=array[i];
	}
	receiveLen=size;
}
#endif

int main(){
	//「#」からはじまるpre-processor命令はここではあまり重要ではないので無視してください
	
	#ifdef ENABLE_motor1_AS_KMD ///////////////////////////////////////////
	//////////////////////////////
	// 部内開発KMDについて
	//////////////////////////////
	
	//名前の再定義方法(参照)
	rob::aKMD &motor=rob::motor1;
	
	//部内開発のKMDの場合、シャットダウンモードに移行するには次のようにします
	//motor.setShutdown(true);
	
	//モータに出力するには次のようします
	//シャットダウンモードの場合起動したうえで出力されます
	motor=0.3;
	
	#elif defined(ENABLE_motor1_AS_ROHMMD) ////////////////////////////////
	//////////////////////////////
	// ROHM社提供のMDについて
	//////////////////////////////
	
	//名前の再定義方法(参照)
	rob::aRohmMD &motor=rob::motor1;
	
	//ROHM社提供のMDの場合、部内のKMDと違いブレーキを用いたPWMはできません
	//しかしPWMを使わなければブレーキできます
	//これを用いてデフォルトでは出力が0.0の時にブレーキをする設定になっており
	//それを無効にするには次のメソッドを呼びます
	motor.setIsBrakeWhenZero(false);
	
	//モータに出力するには次のようします
	motor=0.3;
	
	#endif /////////////////////////////////////////////////////////////////
	
	#ifdef ENABLE_imu03a ///////////////////////////////////////////////////
	//////////////////////////////
	// IMUについて
	//////////////////////////////
	
	//名前の再定義方法(参照)
	rob::a_imu03a &imu=rob::imu03a;
	
	
	//ジャイロのオフセットを取得します
	//マシンが止まっていればるメインループの中でも使えます
	//この場合はマシンが動いていると補正できません
	imu.gyroX.resetOffset();
	
	//前提としてジャイロセンサは角速度を出すセンサーです
	//角速度から角度を出すには時間で積分をする必要があります
	//ライブラリの内部に私が実装してあるのでこれを使いましょう
	
	//積分をリセットするには次のメソッドを呼びます
	//これを呼ばなくてもプログラムが開始した時点では0.0になっているので必ずしも呼ぶ必要はありません
	imu.gyroX.resetDeg();
	//積分を開始するには次のメソッドを呼びます
	imu.gyroX.startDeg();
	//積分を停止するには次のメソッドを呼びます
	//imu03a.gyroX.stopDeg();
	
	#endif /////////////////////////////////////////////////////////////////
	
	////////////////////////////////////////////////////////////////////////
	//////////////////////////////
	// PIDについて
	//////////////////////////////
	
	//定義方法①
	//				ゲイン KP,KI,KD,dt,max,min
	//rob::aPid<float> pid(0.1,0.1,0.01,0.02,1.0,-1.0);
	
	//定義方法②
	//普通の構造体pidGainを用いる方法です　メンバp,i,dがあります
	//				ゲイン KP,KI,KD
	//rob::pidGain pidGain={0.1,0.1,0.01};
	//					  ゲイン,dt,max,min
	//rob::aPid<float> pid(pidGain,0.02,1.0,-1.0);
	
	//ゲインの再定義方法①
	//pid.setGain(0.2,0.2,0.02);
	//ゲインの再定義方法②　定義方法と同じく
	//pid.setGain(pidGain);
	
	/////////////////////////////////////////////////////////////////////////
	
	#ifdef ENABLE_rotaryEncoder1 ////////////////////////////////////////////
	//////////////////////////////
	// エンコーダについて
	//////////////////////////////
	
	//参照を使った名前の再定義方法
	//ライブラリに添付したバージョンでは4倍になります
	//すなわち、一回転で500パルスの設定なら、一回転で2000パルスとして読まれます
	//別のバージョン(1倍、2倍)も作れるので作りたいです（小並感）
	rob::aRotaryEncoder &encoder=rob::rotaryEncoder1;
	
	//回転パルスの再定義および、回転パルスの積算値を初期化して0にします
	encoder.reset();
	
	//今の回転パルスを300と再定義します(補正します)
	//他のセンサ（リミットスイッチなど）と組み合わせるときに有用です
	encoder.set(300);
	
	#endif /////////////////////////////////////////////////////////////////
	
	#ifdef ENABLE_xbeeCore ////////////////////////////////////////////////
	//////////////////////////////
	// xbeeについて
	//////////////////////////////
	
	//別のxbeeモジュールとの間に専用の通信線を用意します
	//ここの例では別のxbeeモジュールの64bitアドレスは0x0013a20040ca9d3bです
	rob::aXbeeCom xbeeTunnel(rob::xbeeCore,rob::xbee64bitAddress(0x00,0x13,0xa2,0x00,0x40,0xCA,0x9D,0x3B));
	
	//上で指定したxbeeモジュールから何かを受け取ったときに動作する関数をattachします
	xbeeTunnel.attach(callback(whenReceiveFunc));
	
	#endif /////////////////////////////////////////////////////////////////
	
	#ifdef ENABLE_switchJ7 /////////////////////////////////////////////////
	//////////////////////////////
	// スイッチについて
	//////////////////////////////
	
	//名前の再定義方法(参照)
	DigitalIn &sw=rob::switchJ7;
	
	#endif /////////////////////////////////////////////////////////////////
	
	rob::regularC_ms actTime(100);
	
	while(true){
		
		
		if(actTime){
			#ifdef ENABLE_imu03a ///////////////////////////////////////////
			//////////////////////////////
			// IMUについて
			//////////////////////////////
			
			//実際にIMUで角度を読むには次のようにします
			float imuGyroXDeg=imu.gyroX.getDeg();
			//角速度を知りたいときには次のようにします
			//なおDDeg=Delta(微分) Degree=角速度
			float imuGyroXDDeg=imu.gyroX.getDDeg();
			pc.printf("gyroX:%4.2f[deg] %4.2f[deg/s] ",imuGyroXDeg,imuGyroXDDeg);
			
			//ジャイロセンサーのほかに角度を知れるセンサーがあり、それでジャイロの値を補正するには次のメソッドを呼びます
			//この場合はマシンが動いていても補正できるはずです
			//ちなみに2020年の秋田Aの竿燈ロボの肝（老害の語り）
			//const float ddeg;//角速度の真値(とできる値)
			//const float mult;//既存のオフセット値にこの真値を適用するときの指数移動平均によるLPFの定数(通常0.01程度)
			//imu.gyroX.calcOffsetByTrueDdeg(ddeg,mult);
			
			//マシンが動いていない(IMUが静止状態である)時には、次のメソッドでジャイロの値を補正できます
			//この場合はマシンが動いていると補正できません
			//imu.gyroX.resetOffset();
			
			
			//IMUで加速度を読みには次のようにします
			//物理で習うm/s^2で加速度を得る方法
			float imuAccelXMperS2=imu.accelX.getMperS2();
			//地球の重力加速度を1としたときのGで得る方法
			//float imuAccelXG=imu.accelX.getG();
			pc.printf("accelX:%4.2f[m/s^2] ",imuAccelXMperS2);
			
			#endif /////////////////////////////////////////////////////////
			//////////////////////////////
			// PIDについて
			//////////////////////////////
			
			//float sensorVal;//センサーの値
			//float pidOutput=pid.calc(sensorVal);
			//pidOutputをモータの出力などにするとよいです
			
			#ifdef ENABLE_rotaryEncoder1 ///////////////////////////////////
			//////////////////////////////
			// エンコーダについて
			//////////////////////////////
			
			//エンコーダの値を取得します
			//operatorを使っているので簡単にかけます
			int encoderVal=encoder;
			//または
			//int encoderVal=encoder.read();
			pc.printf("enc:%d ",encoderVal);
			
			#endif /////////////////////////////////////////////////////////
			
			#ifdef ENABLE_xbeeCore /////////////////////////////////////////
			//////////////////////////////
			// xbeeについて
			//////////////////////////////
			//whenReceiveFuncで受信したデータを表示してみます
			pc.printf("receive:");
			for(int i=0;i<receiveLen;i++){
				pc.printf("0x%X,",receiveArray[i]);
			}
			
			//送るデータです
			uint8_t sendArray[]={0x45,0x33};
			
			//これで送信できます
			xbeeTunnel.send(sendArray,ARRAYLEN(sendArray));
			
			#endif /////////////////////////////////////////////////////////
			
			#ifdef ENABLE_switchJ7 /////////////////////////////////////////
			//////////////////////////////
			// スイッチについて
			//////////////////////////////
			//whenReceiveFuncで受信したデータを表示してみます
			pc.printf(" sw:%s",sw?" on":"off");
			
			#endif /////////////////////////////////////////////////////////
			
			pc.printf("\n");
		}
	}
	
    return 0;
}