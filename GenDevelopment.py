from numpy import random;
from TraitResultObject import TraitResult;

def SinglePoint(a,b):
	Deer1 = a;
	Deer2 = b;
	traits1 = Deer1.traits;
	traits2 = Deer2.traits;
	point = random.randint(len(traits1));
	traitsplit1 = traits1[0:point];
	traitsplit2 = traits2[point:len(traits2)];
	newTrait = traitsplit1+traitsplit2;
	NewDeer = TraitResult();
	NewDeer.assign(0,newTrait,str(0),str(0),[]);
	return NewDeer;

def DoublePoint(a,b):
	Deer1 = a;
	Deer2 = b;
	traits1 = Deer1.traits;
	traits2 = Deer2.traits;
	points = random.choice(len(traits1),2);
	points.sort();
	traitsplit1 = traits1[0:points[0]];
	traitsplit2 = traits2[points[0]:points[1]];
	traitsplit3 = traits1[points[1]:len(traits1)];
	newTrait = traitsplit1 + traitsplit2 + traitsplit3;
	NewDeer = TraitResult();
	NewDeer.assign(0,newTrait,str(0),str(0),[]);
	return NewDeer;

def RandomPoint(a,b):
	newTrait = '';
	Deer1 = a;
	Deer2 = b;
	traits1 = Deer1.traits;
	traits2 = Deer2.traits;
	for x in range(0,len(traits1)):
		choice = random.randint(1);
		if(choice == 0):
			newTrait = newTrait + traits1[x];
		else:
			newTrait = newTrait + traits2[x];
	NewDeer = TraitResult();
	NewDeer.assign(0,newTrait,str(0),str(0),[]);
	return NewDeer;

def AndCross(a,b):
	newTrait = '';
	Deer1 = a;
	Deer2 = b;
	traits1 = Deer1.traits;
	traits2 = Deer2.traits;
	for x in range(0,len(traits1)):
		if(traits1[x] == '1' and traits2[x] == '1'):
			newTrait = newTrait + '1';
		else:
			newTrait = newTrait + '0'
	NewDeer = TraitResult();
	NewDeer.assign(0,newTrait,str(0),str(0),[]);
	return NewDeer;

def OrCross(a,b):
	newTrait = '';
	Deer1 = a;
	Deer2 = b;
	traits1 = Deer1.traits;
	traits2 = Deer2.traits;
	for x in range(0,len(traits1)):
		if(traits1[x] == '1' or traits2[x] == '1'):
			newTrait = newTrait + '1';
		else:
			newTrait = newTrait + '0'
	NewDeer = TraitResult();
	NewDeer.assign(0,newTrait,str(0),str(0),[]);
	return NewDeer;

def Mutate(a):
	newTrait = '';
	Deer1 = a;
	traits1 = Deer1.traits;
	for x in range(0,len(traits1)):
		if(traits1[x] == '1'):
			newTrait = newTrait + '0';
		else:
			newTrait = newTrait + '1'
	NewDeer = TraitResult();
	NewDeer.assign(0,newTrait,str(0),str(0),[]);
	return NewDeer;