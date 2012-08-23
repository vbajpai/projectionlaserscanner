all: build make

build:
	latexmk -pvc -f -pdf -quiet plscanner.tex

clean:
	latexmk -c
	rm -f -r *.lol
	rm -f -r *.bbl
