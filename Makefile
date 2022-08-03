all:
	g++ *.cpp -o res -std=c++14 -O1

clean:
	rm ./res
