public class Human{
	public Human(){
		int age;
		int money;
		String name;
	}
	
	public void setName(String randomName){
		name = randomName;
	}
	
	public String getName(){
		System.out.println("my name is" + name);
		return name;
	}
	
	public static void main(String[] args){
		Human maidi = new Human();
		maidi.setName("Maidi Xu");
		maidi.getName();
	}
	
}
	