#pragma once
class Cell
{
	//public attributes
	public:
		 std::vector<Marker> myAuxins;
		 //position
		 float posX;
		 float posY;
		 float posZ;
		 //name
		 std::string name;
		 //can i pass this cell?
		 //1 = can pass
		 //9 = cannot pass
		 int canPass;

	//public methods
	public:
		Cell();
		Cell(float newPosX, float newPosY, float newPosZ, std::string newName);
		~Cell();
		void StartList();
		void AddAuxin(Marker auxin);
		std::vector<Marker>* GetAuxins();
};

