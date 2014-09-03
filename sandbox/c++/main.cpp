#include <iostream>
#include <string>



using namespace std;



class Object {
	public:
		Object() : _name("Adam") {}
		Object(std::string name) : _name(name) {}
		
		std::string getName() const { return _name; }
	
	private:
		std::string _name;
};



int main(int argc, char* argv[])
{
	Object a;
	Object b();
	
	cout << a.getName() << " " << b.getName() << endl;
	
	return 0;
}
