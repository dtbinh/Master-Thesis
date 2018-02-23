TARGET=example

all: xelatex

fast:
	xelatex $(TARGET).tex

xelatex:
	xelatex $(TARGET).tex 
	-bibtex $(TARGET)
	xelatex $(TARGET).tex
	xelatex $(TARGET).tex

pdflatex:
	pdflatex -shell-escape $(TARGET).tex 
	-bibtex $(TARGET)
	pdflatex -shell-escape $(TARGET).tex
	pdflatex -shell-escape $(TARGET).tex

latex:
	latex $(TARGET).tex 
	-bibtex $(TARGET)
	latex $(TARGET).tex
	latex $(TARGET).tex
	dvipdf $(TARGET).dvi

clean:
	-rm $(TARGET).aux
	-rm $(TARGET).toc
	-rm $(TARGET).out
	-rm $(TARGET).snm
	-rm $(TARGET).nav
	-rm $(TARGET).bbl
	-rm $(TARGET).blg
	-rm $(TARGET).log
	-rm $(TARGET).pdf
	-rm $(TARGET).dvi
	-rm $(TARGET)-blx.bib
	-rm $(TARGET).run.xml
	-rm texput.log
