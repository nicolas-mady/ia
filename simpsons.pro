homem(abraham).
homem(herb).
homem(homer).
homem(clancy).
homem(bart).
homem(yang).
mulher(mona).
mulher(marge).
mulher(jackie).
mulher(patty).
mulher(selma).
mulher(lisa).
mulher(maggie).
mulher(ling).
progenitor(abraham, herb).
progenitor(abraham, homer).
progenitor(clancy, marge).
progenitor(mona, herb).
progenitor(mona, homer).
progenitor(clancy, patty).
progenitor(clancy, selma).
progenitor(jackie, marge).
progenitor(jackie, patty).
progenitor(jackie, selma).
progenitor(homer, bart).
progenitor(homer, maggie).
progenitor(homer, lisa).
progenitor(marge, bart).
progenitor(marge, maggie).
progenitor(marge, lisa).
progenitor(selma, ling).
progenitor(yang, ling).

:- progenitor(homer, X).
:- findall(X, progenitor(homer, X), Filhos).
:- forall(progenitor(homer, X), (write('Homer é pai de: '), writeln(X))).