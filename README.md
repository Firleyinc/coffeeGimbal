# coffeeGimbal
Projekt zaliczeniowy z przedmiotu Teoria Sterowania w Robotyce (II stopień, AiR).

![ccc overview](https://github.com/user-attachments/assets/990dcdd5-9990-4b04-be5f-73600949209d)

Ideą jest stworzenie urządzenia stabilizującego ruchy płynu wewnątrz kubka, przeciwdziałającego jego wylewaniu się przy zmianach położenia w płaszczyźnie poziomej (X-Y) i orientacji.

## W skład niniejszego projektu wchodzą dwie części - symulator i sterownik.

1. Wewnątrz środowiska symulacyjnego ma zostać zaimplementowana mechanika urządzenia - wklęsłego talerza, zawieszonego na dwóch przegubach obrotowych, umożliwiających zadawanie dowolnego położenia w osi X i Y, w zakresie od -90 do +90 stopni. Obiektem do utrzymywania na talerzu będzie kula, swobodnie przemieszczająca się w przestrzeni symulatora - zadaniem sterownika jest wytworzenie takich sterowań, żeby w trakcie zmian położenia i orientacji układu globalnego, kula znajdowała się możliwie jak najbliżej środka talerza, a wektor sił wynikających z jej bezwładności, był skierowany wył osi -Z w układzie współrzędnych pojemnika. Docelowo kula może zostać zastąpiona dokładniejszym modelem, działającym zgodnie z zasadami dynamiki płynów, a talerz zastąpiony kubkiem, pod warunkiem rozszerzenia wektora stanów. Wybrane środowisko symulacyjne są przyspieszenie, oraz zryw kulki
2. Celem jest zaimplementowanie sterownika mającego za zadanie talerzwać ruchy globalnego układu współrzędnych i minimalizować siły w kierunku promieniowym, działające na kulkę na talerzu. Sygnałami wejściowymi są przyspieszenie, oraz zryw kulki, natomiast wyjściowymi - położenia przegubów układu talerzącego talerz. Na potrzeby realizacji zadania, zakłada się, że siły w osiach Z nie występują i w związku z tym nie ma konieczności ich kompensowania - innymi słowy stabilizujemy talerz na umieszczony na statku, a nie w myśliwcu.

## Struktura projektu

1. Model symulacji załadowywany do symulatora Mujoco znajduje się w pliku 'gimbal_simplified.xml' w katalogu /simulation. Definiuje on zarówno wizualizacje jak i model fizyczny gimballa.

2. Główny plik wykonywalny to 'main.py' w katalogu /python. Plik ten jest odpowiedzialny za:
- główny skrypt uruchamiający symulator Mujoco
- definicja generatora trajektorii sinusoidalnych używanych w testach układu
- pozyskiwanie i wysyłanie parametrów symulacji
- używanie zaimplementowanego regulatora do regulacji kąta wychylenia talerza
- definicja okna zawierającego wykresy wartości (przemieszczenia, przyspieszenia, zrywu, wychylenia talerza); oraz checkboxy załączające odpowiednie fynkcjonalnosci sterownika i generatorów trajektorii
- definicja zależności suwaków pozycyjnych w symulatorze Mujoco służących, jako kolejny sposób generowania trajektorii, ale tym razem wedle uznania użytkownika

3. Regulator znajduje się w pliku 'controller.py' w katalogu /python zaimplementowanym regulatorem jest regulator typu P, gdyż dawał nam on najlepsze wyniki.

## Opis rozwiązania
Symulator Mujoco, definiując aktuatory implementuje do każdego z nich wewnętrzny regulator PID, do którego uzytkownik nie ma dostępu. W skutek testów mogliśmy zaobserwować, że regulator P jest w stanie współgrać z wewnętrznymi regulatorami symulatora Mujoco, tzn. regulatory nie zakłócają się na wzajem.
Ważnym krokiem było także nadanie masy ramionom systemu Gimbal. Pozwoliło nam to nadać głównemu układowi pewien moment inercji, przez co ograniczyliśmy wpływ kuli na cału układ.
