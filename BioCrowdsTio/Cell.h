#pragma once
class Cell
{
	//public attributes
	public:
		 std::vector<Marker> myAuxins;
		 //position
		 Vector3 position;
		 //name
		 std::string name;
		 //can i pass this cell?
		 //1 = can pass
		 //9 = cannot pass
		 int canPass;

	//public methods
	public:
		Cell();
		Cell(Vector3 newPosition, std::string newName);
		~Cell();
		void StartList();
		void AddAuxin(Marker auxin);
		std::vector<Marker>* GetAuxins();
};

