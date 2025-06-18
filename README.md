# coffeeGimbal
Projekt zaliczeniowy z przedmiotu Teoria Sterowania w Robotyce (II stopień, AiR).

![ccc overview](https://github.com/user-attachments/assets/990dcdd5-9990-4b04-be5f-73600949209d)

Ideą jest stworzenie urządzenia stabilizującego ruchy płynu wewnątrz kubka, przeciwdziałającego jego wylewaniu się przy zmianach położenia w płaszczyźnie poziomej (X-Y) i orientacji.

## W skład niniejszego projektu wchodzą dwie części - symulator i sterownik.

1. Wewnątrz środowiska symulacyjnego ma zostać zaimplementowana mechanika urządzenia - wklęsłego talerza, zawieszonego na dwóch przegubach obrotowych, umożliwiających zadawanie dowolnego położenia w osi X i Y, w zakresie od -90 do +90 stopni. Obiektem do utrzymywania na talerzu będzie kula, swobodnie przemieszczająca się w przestrzeni symulatora - zadaniem sterownika jest wytworzenie takich sterowań, żeby w trakcie zmian położenia i orientacji układu globalnego, kula znajdowała się możliwie jak najbliżej środka talerza, a wektor sił wynikających z jej bezwładności, był skierowany wył osi -Z w układzie współrzędnych pojemnika. Docelowo kula może zostać zastąpiona dokładniejszym modelem, działającym zgodnie z zasadami dynamiki płynów, a talerz zastąpiony kubkiem, pod warunkiem rozszerzenia wektora stanów. 
2. Celem pierwszego etapu jest zaimplementowanie środowiska symulacyjnego oraz sterownika mającego za zadanie minimalizować przemieszczenia kulki na "talerzu" (podstawie) względem jego lokalnego układu współrzędnych. Sygnałami wejściowymi są przyspieszenie oraz zryw talerza w płaszczyźnie X-Y, natomiast wyjściowymi - położenia kątowe theta talerza. Na potrzeby realizacji zadania, zakłada się, że przyspieszenie w osi Z układu globalnego nie zmienia się i w związku z tym nie ma konieczności jego kompensowania.

## Struktura projektu

1. Wszelkie pliki modelu niezbędnego do symulacji znajdują się w folderze 'simulation' - są to bryły oraz plik szablonu MuJoCo gimbal_simplified.xml. Ten z kolei definiuje zarówno część graficzną wizualizacji, jak i parametry modelu dynamiki układu.

2. Pliki symulatora i regulatora znajdują się katalogu 'python'. Symulację uruchamia się z poziomu pliku main.py.
- main.py: skrypt pętli głównej programu, podstawowe parametry konfiguracyjne, definicje obiektów;
- traj_gen.py: klasa generatora trajektorii, służącego do opcjonalnego wytwarzania sygnałów wejściowych, np. na potrzeby oceny jakości regulacji;
- controller.py: klasy implementujące pojedynczy regulator kompensujący przyspieszenia talerza jego wychyleniem;
- mujoco_interface.py: klasy interfejsu Python <-> MuJoCo. Odpowiadają za sterowanie symulacją i dostarczają wektorów stanu i sterowań.
- qtgraph_interface.py: klasy odpowiadające za rysowanie interfejsu GUI, przedstawiającego zmiany wybranych parametrów w czasie rzeczywistym, w formie wykresów. Do rysowania GUI wykorzystano bibliotekę PyQt6 oraz PyQtGraph.
- requirements.txt: zawiera spis wymaganych bibliotek zewnętrznych. 


## Opis rozwiązania

![til](.gimball.gif)
