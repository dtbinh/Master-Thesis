\ProvidesClass{LSR_thesis}[2016/04/05 class for LSR slides]
\typeout{-- 2917/07/24 revision V0.3 by D.Wollherr}
\typeout{-- 2017/01/17 revision V0.2 by V.Gabler}
\typeout{-- 2016/04/05 revision V0.1 by V.Gabler}
\typeout{-- This is early beta.}
\NeedsTeXFormat{LaTeX2e}

% define flags in use
\newif\if@langGer      \global\@langGertrue
\newif\if@typeFinal      \global\@typeFinaltrue
\newif\if@addAddress   \global\@addAddresstrue

% define style input options
\DeclareOption{LSR}{\def\@institute{LSR}}%
\DeclareOption{ITR}{\def\@institute{ITR}}%
\DeclareOption{german}{\@langGertrue}
\DeclareOption{english}{\@langGerfalse}
\DeclareOption{final}{\@typeFinaltrue}
\DeclareOption{intermediate}{\@typeFinalfalse}
\DeclareOption{BA}{\def\@worktype{BA}}%
\DeclareOption{DA}{\def\@worktype{DA}}%
\DeclareOption{MA}{\def\@worktype{MA}}%
\DeclareOption{HS}{\def\@worktype{HS}}%
\DeclareOption{PP}{\def\@worktype{PP}}%
\DeclareOption{SeA}{\def\@worktype{SeA}}%
\DeclareOption{IP}{\def\@worktype{IP}}%
\DeclareOption{FP}{\def\@worktype{FP}}%
\DeclareOption{homeless}{\@addAddressfalse}


% default to german
\ExecuteOptions{german}
% default to final submission
\ExecuteOptions{final}
% make it into effect
\ProcessOptions

% so far only standard format
\LoadClass[a4paper,twoside, openright,12pt]{report}

% load packages
\RequirePackage{subfigure}
\RequirePackage{verbatim}
\RequirePackage{psfrag,amsmath,amssymb,amsbsy,graphics,float}
\RequirePackage{graphicx, color, soul}
\RequirePackage{ifthen}
\RequirePackage{anyfontsize}
\if@langGer
  \RequirePackage[ngerman]{babel}
  \RequirePackage[utf8]{inputenc}
\else
  \RequirePackage[latin1]{inputenc}
\fi

% some helper variables for \ifx
\def\@worktypeDA{DA}
\def\@worktypeBA{BA}
\def\@worktypeMA{MA}
\def\@worktypeHS{HS}
\def\@worktypePP{PP}
\def\@worktypeSeA{SeA}
\def\@worktypeIP{IP}
\def\@worktypeFP{FP}
\def\@instituteLSR{LSR}
\def\@instituteITR{ITR}

\newdimen\LSRx
\newdimen\LSRzero
\newdimen\LSRborder
\LSRborder 6mm
\LSRx 10mm
\LSRzero 10pt


%_______Kopf- und Fußzeile_______________________________________________________
\RequirePackage{fancyhdr}
\pagestyle{fancy}
%um Kopf- und Fußzeile bei chapter-Seiten zu reaktivieren
\newcommand{\helv}{%
   \fontfamily{phv}\fontseries{a}\fontsize{9}{11}\selectfont}
\fancypagestyle{plain}{	
	\fancyfoot{}% keine Fußzeile
	\fancyhead[RE]{\helv\leftmark}% Rechts auf geraden Seiten=innen; in \leftmark stehen \chapters
	\fancyhead[LO]{\helv\rightmark}% Links auf ungeraden Seiten=außen;in \rightmark stehen \sections
	\fancyhead[RO,LE]{\thepage}}%Rechts auf ungeraden und links auf geraden Seiten
%Kopf- und Fußzeile für alle anderen Seiten
\fancyfoot{}
\fancyhead[RE]{\helv\leftmark}
\fancyhead[LO]{\helv\rightmark}%alt:\fancyhead[LO]{\itshape\rightmark}
\fancyhead[RO,LE]{\thepage}
%________________________________________________________________________________


%_Definieren der Ränder und Längen__________
\setlength{\textwidth}{15cm}
\setlength{\textheight}{22cm}
\setlength{\evensidemargin}{-2mm}
\setlength{\oddsidemargin}{11mm}
\setlength{\headwidth}{15cm}
\setlength{\topmargin}{10mm}
\setlength{\parindent}{0pt} % Kein Einrücken beim Absatz!!
%___________________________________________

%_Hyperref for CC Url__________
\RequirePackage{hyperref}
%___________________________________________


%_________select correct heading________________
\if@langGer
  \typeout{-- language set to german.}
  \ifx\@worktype\@worktypeDA
    \def\@worktypetext{DIPLOMARBEIT}
    \typeout{-- Type: Diplomarbeit.}
  \else
  \ifx\@worktype\@worktypeBA
    \def\@worktypetext{BACHELORARBEIT}
    \typeout{-- Type: Bachelorarbeit.}
  \else
  \ifx\@worktype\@worktypeMA
    \def\@worktypetext{MASTERARBEIT}
    \typeout{-- Type: Masterarbeit.}
  \else
  \ifx\@worktype\@worktypeHS
    \def\@worktypetext{HAUPTSEMINAR}
    \typeout{-- Type: Hauptseminar.}
  \else
  \ifx\@worktype\@worktypePP
    \def\@worktypetext{PROJEKTPRAKTIKUM}
    \typeout{-- Type: Projektpraktikum.}
  \else
  \ifx\@worktype\@worktypeSeA
    \def\@worktypetext{SEMESTERARBEIT}
    \typeout{-- Type: Semesterarbeit.}
  \else
  \ifx\@worktype\@worktypeIP
    \def\@worktypetext{INGENIEURPRAXIS}
    \typeout{-- Type: Industriepraxis.}
  \else
  \ifx\@worktype\@worktypeFP
    \def\@worktypetext{FORSCHUNGSPRAXIS}
    \typeout{-- Type: Forschungspraxis.}
  \else
    \def\@worktypetext{BACHELORARBEIT/MASTERARBEIT}
    \typeout{-- Type: not specified.}
  \fi
  \fi
  \fi
  \fi
  \fi
  \fi
  \fi
  \fi
\else % english
  \typeout{-- language set to english.}
  \ifx\@worktype\@worktypeDA
    \def\@worktypetext{DIPLOMA THESIS}
    \typeout{-- Type: Diplomarbeit.}
  \else
  \ifx\@worktype\@worktypeBA
    \def\@worktypetext{BACHELOR'S THESIS}
    \typeout{-- Type: Bachelorarbeit.}
  \else
  \ifx\@worktype\@worktypeMA
    \def\@worktypetext{MASTER'S THESIS}
    \typeout{-- Type: Masterarbeit.}
  \else
  \ifx\@worktype\@worktypeHS
    \def\@worktypetext{SCIENTIFIC SEMINAR}
    \typeout{-- Type: Scientific Seminar.}
  \else
  \ifx\@worktype\@worktypePP
    \def\@worktypetext{PRACTICAL COURSE}
    \typeout{-- Type: Projektpraktikum.} 
  \else
  \ifx\@worktype\@worktypeSeA
    \def\@worktypetext{SEMESTER THESIS}
    \typeout{-- Type: Semesterarbeit.}
  \else
  \ifx\@worktype\@worktypeIP
    \def\@worktypetext{INGENIEURPRAXIS}
    \typeout{-- Type: Industriepraxis.}
  \else
  \ifx\@worktype\@worktypeFP
    \def\@worktypetext{FORSCHUNGSPRAXIS}
    \typeout{-- Type: Forschungspraxis.}
  \else
    \def\@worktypetext{FINAL THESIS}
    \typeout{-- Type: not specified.}
    \fi
    \fi
    \fi
    \fi
    \fi
    \fi
    \fi
    \fi
    \fi
%___________________________________________

% 
%___________the student data variables______________
\def\@student{N.N.}
\newcommand{\student}[1]{\def\@student{\mbox{}#1}}
\def\@vartitle{}
\newcommand{\studtitle}[1]{\def\@vartitle{#1}}
\def\@studenttwo{\mbox{}}
\newcommand{\studenttwo}[1]{\def\@studenttwo{\mbox{}#1}}
\def\@vartitletwo{}
\newcommand{\studtitletwo}[1]{\def\@vartitletwo{#1}}
\def\@studentthree{\mbox{}}
\newcommand{\studentthree}[1]{\def\@studentthree{\mbox{}#1}}
\def\@vartitlethree{\mbox{}}
\newcommand{\studtitlethree}[1]{\def\@vartitlethree{\mbox{}#1}}
\def\@studentfour{\mbox{}}
\newcommand{\studentfour}[1]{\def\@studentfour{\mbox{}#1}}
\def\@vartitlefour{\mbox{}}
\newcommand{\studtitlefour}[1]{\def\@vartitlefour{\mbox{}#1}}

\def\@varyearone{DD. MM. YYYY}
\newcommand{\yearofbirth}[1]{\def\@varyearone{#1}}
\def\@varstreetone{Bakerstreet 221B}
\newcommand{\street}[1]{\def\@varstreetone{#1}}
\def\@varcity{8xxxx München}
\newcommand{\city}[1]{\def\@varcity{#1}}
\def\@vartele{089 - 1234567}
\newcommand{\phone}[1]{\def\@vartele{#1}}
% todo: additional informations for multiple students
%_____________________________________________________
\ifx\@institute\@instituteLSR
\def\@varprofessor{M. Buss}
\else
\ifx\@institute\@instituteITR
\def\@varprofessor{S. Hirche}
\fi
\fi
\newcommand{\professor}[1]{\def\@varprofessor{\mbox{}#1}}
\def\@varprofessortype{Univ.-Professor}
\newcommand{\professortype}[1]{\def\@varprofessortype{\mbox{}#1}}

\def\@vartitletranslation{}
\newcommand{\titletranslation}[1]{\def\@vartitletranslation{#1}}


\def\@varsupervisor{Dipl.-Ing. N.N.}
\newcommand{\supervisor}[1]{\def\@varsupervisor{#1}}

\def\@varstart{xx.xx.xxxx}
\newcommand{\start}[1]{\def\@varstart{#1}}

\def\@varprog{xx.xx.xxxx}
\newcommand{\progrep}[1]{\def\@varprog{#1}}

\def\@varfinal{xx.xx.xxxx}
\newcommand{\finalrep}[1]{\def\@varfinal{#1}}






%___________________footer______________________

%%%%%%%%%%%%%%%%%%%%% F O O T E R %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
\if@langGer
  \ifx\@worktype\@worktypeHS
  \newcommand{\footer}{
    Wissenschaftliche Arbeit im Rahmen des Hauptseminars\\
    an der Fakult\"at f\"ur Elektrotechnik und Informationstechnik der\\ Technischen Universit\"at M\"unchen.
  }
  \else
  \ifx\@worktype\@worktypePP
  \newcommand{\footer}{
    Wissenschaftliche Arbeit im Rahmen des Projektpraktikums\\
    an der Fakult\"at f\"ur Elektrotechnik und Informationstechnik der\\ Technischen Universit\"at M\"unchen.
  }
  \else
  \ifx\@worktype\@worktypeIP
  \newcommand{\footer}{
    Wissenschaftliche Arbeit im Rahmen der Ingenieurspraxis\\
    an der Fakult\"at f\"ur Elektrotechnik und Informationstechnik der\\ Technischen Universit\"at M\"unchen.
  }
  \else
  \ifx\@worktype\@worktypeFP
  \newcommand{\footer}{
    Wissenschaftliche Arbeit im Rahmen der Forschungspraxis\\
    an der Fakult\"at f\"ur Elektrotechnik und Informationstechnik der\\ Technischen Universit\"at M\"unchen.
  }
  \else
  \ifx\@worktype\@worktypeSeA
  \newcommand{\footer}{
    Wissenschaftliche Arbeit im Rahmen der Semesterarbeit\\
    an der Fakult\"at f\"ur Elektrotechnik und Informationstechnik der\\ Technischen Universit\"at M\"unchen.
  }
  \else
  \ifx\@worktype\@worktypeMA
  \newcommand{\footer}{
    Wissenschaftliche Arbeit zur Erlangung des Grades M.Sc.\\
%    \if@typeFinal \else(Zwischenbericht) \\ \fi
    an der Fakult\"at f\"ur Elektrotechnik und Informationstechnik der\\ Technischen Universit\"at M\"unchen.
  }
  \else
  \ifx\@worktype\@worktypeBA
  \newcommand{\footer}{
    Wissenschaftliche Arbeit zur Erlangung des Grades B.Sc.\\
%    \if@typeFinal \else(Zwischenbericht) \\ \fi
    an der Fakult\"at f\"ur Elektrotechnik und Informationstechnik der\\ Technischen Universit\"at M\"unchen.
  }
  \else
  \newcommand{\footer}{
    Unbekannte wissenschaftliche Arbeit
  }
  \fi % BA
  \fi % MA
  \fi % SeA
  \fi % FP
  \fi % IP
  \fi % PP
  \fi % HS
\else
  \ifx\@worktype\@worktypeHS
  \newcommand{\footer}{
    Scientific work within the scientific seminar\\
    from the Department of Electrical and Computer Engineering at the \\ Technical University of Munich.
  }
  \else
  \ifx\@worktype\@worktypePP
  \newcommand{\footer}{
    Scientific work within the practical course\\
    from the Department of Electrical and Computer Engineering at the \\ Technical University of Munich.
  }
  \else
  \ifx\@worktype\@worktypeIP
  \newcommand{\footer}{
    Scientific work within the Ingenieurspraxis\\
    from the Department of Electrical and Computer Engineering at the \\ Technical University of Munich.
  }
  \else
  \ifx\@worktype\@worktypeFP
  \newcommand{\footer}{
    Scientific work within the Forschungspraxis\\
    from the Department of Electrical and Computer Engineering at the \\ Technical University of Munich.
  }
  \else
  \ifx\@worktype\@worktypeSeA
  \newcommand{\footer}{
    Scientific work within the semester thesis\\
    from the Department of Electrical and Computer Engineering at the \\ Technical University of Munich.
  }
  \else
  \ifx\@worktype\@worktypeMA
  \newcommand{\footer}{
    Scientific Thesis for the procurance of the degree M.Sc.\\
    \if@typeFinal \else(Intermediate report) \\ \fi
    from the Department of Electrical and Computer Engineering at the \\ Technical University of Munich.
  }
  \else
  \ifx\@worktype\@worktypeBA
  \newcommand{\footer}{
    Scientific thesis for the procurance of the degree B.Sc.\\
    \if@typeFinal \else(Intermediate report) \\ \fi
    from the Department of Electrical and Computer Engineering at the \\ Technical University of Munich.
  }
  \else
  \newcommand{\footer}{
    Unknown scientific thesis
  }
  \fi % BA
  \fi % MA
  \fi % SeA
  \fi % FP
  \fi % IP
  \fi % PP
  \fi % HS
  \fi % Ger
\pagestyle{empty}
%%%%%%%%%%%%%%%%%%%%% M A K E T I T L E %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
\renewcommand{\maketitle}{
\vspace*{-3cm}
\hfill\includegraphics{./logos/tum_logo.pdf}%
\vspace*{1.2cm}
\begin{minipage}[t][10cm][t]{\textwidth}
\sffamily
{\fontsize{24pt}{26pt}\selectfont\textbf{\@title}}

\vspace*{14pt}
{\fontsize{18pt}{27pt}\selectfont\textbf{\@vartitletranslation}}

\end{minipage}

\begin{minipage}[t][4cm][t]{\textwidth}
\large\sffamily\selectfont
\footer
\end{minipage}

\begin{minipage}{\textwidth}
\normalsize\sffamily\selectfont

\if@langGer
  \begin{tabbing}
  \hspace*{4cm} \= blub \kill
  \textbf{Betreut von} \>
    \begin{minipage}[t]{10cm}
      \ifx\@institute\@instituteLSR
        Univ.-Prof.~Dr.-Ing./Univ.~Tokio Martin Buss\\
        \@varsupervisor\\
        Lehrstuhl f\"ur Steuerungs- und Regelungstechnik\strut
      \fi
      \ifx\@institute\@instituteITR
        Univ.-Prof.~Dr.-Ing.~Sandra Hirche\\
        \@varsupervisor\\
        Lehrstuhl f\"ur Informationstechnische Regelung\strut
      \fi
    \end{minipage}\\[4.3mm]

  \textbf{Eingereicht von}\>
    \begin{minipage}[t]{8cm}
      \@vartitle~\@student\\
      \@varstreetone\\
      \@varcity\\
      \@vartele
    \end{minipage}\\[2mm]

  \textbf{Eingereicht am}\>
    \begin{minipage}[t]{10cm}
      M\"unchen, den \@varfinal\strut
    \end{minipage}
  \end{tabbing}
\else % english
  \begin{tabbing}
  \hspace*{4cm} \= blub \kill
  \textbf{Supervised by} \>
    \begin{minipage}[t]{10cm}
      \ifx\@institute\@instituteLSR
        Univ.-Prof.~Dr.-Ing./Univ.~Tokio Martin Buss\\
        \@varsupervisor\\
        Chair of Automatic Control Engineering\strut
      \fi
      \ifx\@institute\@instituteITR
        Univ.-Prof.~Dr.-Ing.~Sandra Hirche\\
        \@varsupervisor\\
        Chair of Information-Oriented Control\strut
      \fi
    \end{minipage}\\[4.3mm]

  \textbf{Submitted by}\>
    \begin{minipage}[t]{8cm}
      \@vartitle~\@student\\
      \@varstreetone\\
      \@varcity\\
      \@vartele
    \end{minipage}\\[2mm]

  \textbf{Submitted on}\>
    \begin{minipage}[t]{10cm}
      Munich, \@varfinal\strut
    \end{minipage}
  \end{tabbing}
\fi
\end{minipage}
}


%____________________________________________________________
