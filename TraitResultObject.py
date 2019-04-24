class TraitResult:
	
    def __init__(self):
        self.id = 0
        self.traits = 0
        self.result = 0
        self.collisions = 0
        self.resultVec = []


    def assign (self,a,b,c,d,e):
        self.id = a
        self.traits = b
        self.result = c
        self.collisions = d
        self.resultVec = e
