# coffeeGimbal

![ccc overview](https://github.com/user-attachments/assets/990dcdd5-9990-4b04-be5f-73600949209d)

Projekt zaliczeniowy z przedmiotu Teoria Sterowania w Robotyce (II stopień, AiR).
Ideą jest stworzenie urządzenia stabilizującego ruchy płynu wewnątrz kubka, przeciwdziałającego jego wylewaniu się przy zmianach położenia w płaszczyźnie poziomej (X-Y) i orientacji.

W skład niniejszego projektu wchodzą dwie części - symulator i sterownik.

1. Wewnątrz środowiska symulacyjnego ma zostać zaimplementowana mechanika urządzenia - pojemnika na ciecz w kształcie walca otwartego z wierzchu, zawieszonego na dwóch przegubach obrotowych, umożliwiających zadawanie dowolnego położenia w osi X i Y, w zakresie od -90 do +90 stopni. Na potrzeby prototypu, ciecz będzie reprezentowana przez kulę, swobodnie przemieszczająca się wewnątrz pojemnika - zadaniem sterownika jest wytworzenie takich sterowań, żeby w trakcie zmian położenia i orientacji układu globalnego, kula znajdowała się możliwie jak najbliżej środka walca, a wektor sił wynikających z jej bezwładności, był skierowany wył osi -Z w układzie współrzędnych pojemnika. Docelowo kula może zostać zastąpiona dokładniejszym modelem, działającym zgodnie z zasadami dynamiki płynów. Wybrane środowisko symulacyjne to SoftGym https://sites.google.com/view/softgym lub MuJoCo https://mujoco.org.

2. 
