
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.utility.Delay;
import java.util.*;


public class movement1 {
	RegulatedMotor motorR = new EV3LargeRegulatedMotor(MotorPort.C);
	RegulatedMotor motorL = new EV3LargeRegulatedMotor(MotorPort.D);
	
	EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S1);
	SensorMode color = colorSensor.getRGBMode();
	float[] sample = new float[color.sampleSize()];
			
	EV3TouchSensor touchSensor = new EV3TouchSensor(SensorPort.S2);
	SensorMode touch = touchSensor.getTouchMode();
	float[] touchSample = new float[touch.sampleSize()];

	EV3GyroSensor gyroSensor = new EV3GyroSensor(SensorPort.S4);
	SampleProvider gyro = gyroSensor.getAngleMode();
	float[] gyroSample = new float[gyro.sampleSize()];
	
	DifferentialPilot pilot = new DifferentialPilot(3f, 7.7f, motorL, motorR, true); 
	
    static int[] colorArray = {0,0,1,0,0,0,1,1,0,0,1,0,0,0,1,1,0,0,0,1,1,0,0,1,0,0,0,1,1,0,0,1,0,0,0,1,1};

	static int number_Of_Grids = 37;
	static float sensor_Work_P = 0.95f;
	static float sensor_Wrong_P = 0.05f;
	static float move_Work_P = 0.95f;
	static float move_Wrong_P = 0.05f;

	int accurateAngel = 0;
    
	public static void main(String[] args){

		movement1 m = new movement1();
		int pos = 0;
		float[] P = new float[number_Of_Grids];
		for(int x = 0;x < number_Of_Grids;x++){
			P[x] = 1f/(number_Of_Grids);
		}
		
		for(int x = 0;x < 20;x++){
			P = m.moveAndUpdate(m.normalization(m.changeToP(P,m.compare())));
			if(m.maxOfArray(P) > 0.55 && m.numberOfMax(P,m.maxOfArray(P)) == 1){
				pos = m.indexOfMax(P,m.maxOfArray(P));
				System.out.println("S" + m.maxOfArray(P) + "pos" + m.indexOfMax(P,m.maxOfArray(P)));
				break;
			}
			System.out.println(m.maxOfArray(P));
		}
		
		Node initialNode = new Node(4, 7);
	    Node finalNode = new Node(0, 0);
	    
	    int rows = 5;
	    int cols = 8; 
	    AStar aStar = new AStar(rows, cols, initialNode, finalNode);
	    int[][] blocksArray = new int[][]{{0, 3}, {0, 4}, {0, 5},{0,6},{0,7}};
	    aStar.setBlocks(blocksArray);
	    List<Node> path = aStar.findPath();
	    for (Node node : path) {
	        System.out.println(node);
	    }
	    
	    Delay.msDelay(1000);
	    
	    int colChange = initialNode.getCol()-finalNode.getCol();
	    int rowChange = initialNode.getRow()-finalNode.getRow();
	    
	    /*double angle = Math.atan((colChange)/(rowChange));
	    m.rotate(-60);
	    float distance = (float) Math.sqrt(colChange * colChange + rowChange * rowChange)*5;
	    m.moveDistance(distance);
	    m.rotate(63);
	    m.moveDistance(48);
	    m.rotate(45);
	    m.moveDistance(15); */
	     
	 
		m.rotate(-78);
		m.moveDistance(57);
		m.rotate(90);
		m.moveDistance(36);
		m.rotate(40);
		m.moveDistance(35);
		 
		m.fetchTouch();
		Sound.beep(); 
		m.redOrGreen();

	    //Delay.msDelay(50000);
	}
	
	private void move2(){
		motorL.setSpeed(200);
		motorR.setSpeed(200);
		motorL.rotate(65,true); 
		motorR.rotate(65);
	}

	private void fetchTouch(){
		touch.fetchSample(touchSample,0);
		while(touchSample[0] == 0){			
			move2();
			touch.fetchSample(touchSample,0);
		}
	}
	
	private void redOrGreen(){
		color = colorSensor.getRedMode();
		color.fetchSample(sample, 0);
		//G
		if(sample[0] < 0.3){
			moveDistance(-38);
	    	rotate(45);
	    	moveDistance(74);
	    	rotate(90);
	    	moveDistance(50);
	    	rotate(45);
	    	moveDistance(20);
	    	rotate(-62);
	    	moveDistance(5);
	    	fetchTouch();
		}
		//R 
		else {
			moveDistance(-38);
			rotate(45);
			moveDistance(90);
			rotate(45);
			moveDistance(8);
			rotate(45);
			moveDistance(30);
			rotate(90);
			moveDistance(37);
			accurateAngel = -90;
			rotate(-103);
			moveDistance(40);
			fetchTouch();
		}
	}
	
	private void rotateM(){
		gyro.fetchSample(gyroSample,0);
		if(accurateAngel > gyroSample[0]){
			pilot.rotate(abs(abs(accurateAngel) - abs(gyroSample[0])));
		}
		else if (accurateAngel < gyroSample[0])
		{
			pilot.rotate(-abs(abs(accurateAngel) - abs(gyroSample[0])));
		}
		else {
			System.out.println("succ");
		}
	}
	
	private float abs(float x){
		if(x >= 0) {return x;}
		else {return (-x);}
	}

	private void moveDistance(float distance){
		int distanceP = (int)((distance/9.4245)*360);
		motorL.setSpeed(500); 
		motorR.setSpeed(500);
		motorL.rotate(distanceP,true); 
		motorR.rotate(distanceP);
	}
	
	private void rotate(float angel){
		accurateAngel += angel;
		motorL.setSpeed(200);
		motorR.setSpeed(200);
		pilot.rotate(angel);
		rotateM();
	}
	
	private int fetchColor(){
		color.fetchSample(sample, 0);
		if(sample[0]+sample[1]+sample[2]<0.2 && sample[0]+sample[1]+sample[2]>0.1){
			return 0; //blue
		}
		else {
			return 1; //white
		}
	}
	
	private float[] compare(){
		float[] temp = new float[number_Of_Grids];
		int color = fetchColor();
		for(int x = 0;x < number_Of_Grids;x++){
			temp[x] = color;
		}
		for(int x = 0;x < number_Of_Grids;x++){
			if(temp[x] == colorArray[x]){
				temp[x] = 1;
			}
			else{
				temp[x] = 0;
			}
		}
		return temp;
	}
	
	private float[] changeToP(float[] old,float[] compareP){
		for(int x = 0; x < compareP.length;x++){
			if(compareP[x] == 1.0){
				old[x] *= sensor_Work_P;
			}
			else {
				old[x] *= sensor_Wrong_P;
			}
		}
		return old;
	}

	private float[] normalization(float[] temp){
		float base = sumOfArray(temp);
		for(int x = 0;x < temp.length;x++){
			temp[x] = temp[x]/base;
		}
		return temp;
	}

	private float sumOfArray(float[] temp){
		float sum = 0f;
		for(float i : temp) {
			sum += i;
		}
		return sum;
	}

	private float maxOfArray(float[] temp){
		float max;
		max = temp[0];
		for (int x = 1;x < temp.length;x++) {
			if(temp[x] > max){
				max = temp[x];
			}
		}
		return max;
	}
	
	private int numberOfMax(float[] array, float max){
		int counter = 0;
		for(int x = 0;x < array.length;x++){
			if(array[x] == max){
				counter++;
			}
		}
		return counter;
	}
	
	private int indexOfMax(float[] array, float max){
		int index = 0;
		for(int x = 0;x < array.length;x++){
			if(array[x] == max){
				index = x;
			}
		}
		return index;
	}

	private float[] moveAndUpdate(float[] temp){
		move2();
		float start = temp[0];
		for(int x = 0; x < temp.length -1;x++){
			temp[x] = move_Work_P * temp[x+1] + move_Wrong_P * temp[x];
		}
		temp[temp.length-1] =  move_Work_P * start + move_Wrong_P * temp[temp.length-1];
		return temp;
	}
	
	private int getAccurateAngel(){
		return accurateAngel;
	}
	private float getAngel(){
		gyro.fetchSample(gyroSample,0);
		return gyroSample[0];
	}
	
}