all: build make

build:
	latexmk -pvc -pdf -quiet vbajpai-thesis.tex

clean:
	latexmk -c
	rm -f -r *.lol
	rm -f -r *.bbl
