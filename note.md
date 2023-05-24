# Parachutes Swarm Project 
## Obiettivi
Condurre il centroide di uno sciame di paracaduti verso la posizione desiderata. Alcuni
paracadute sono equipaggiati con sensori GPS, in modo tale da calcolare la propria posiziona
assoluta, gli altri possono solo misurare la propria posizione relativa rispetto agli altri paracadute
all’interno del sensing range. Implementare il Kalman Filter (o EKF) in modo tale che:
- Il centroide del gruppo raggiunga il target: la legge di controllo deve fare in modo che il drone si
muova in modo tale ta muovere in un certo modo il centroide;
- Un paracadute sia designato ad arrivare al target (con o senza GPS) e tutti gli altri devono solo
stargli vicino senza allontanarsi troppo.
Il KF va implementato in modo distribuito in modo da ottenere l’avarage consensus.

## Procedimento
1. Dinamica del paracadute
2. KF con un drone con GPS con traiettoria predefinita (o PID)
3. Distributed KF senza vincoli di posizione
4. Aggiunta di vincoli di posizione con Voronoi
5. Controllo ottimo sul centroide:
- O uso come stato di controllo la posizione del centroide come funzione della posizione dei
paracadute (e quindi ottengo direttamente gli input da dare ai paracadute);
- Oppure tengo come stato di controllo la posizione implicita del centroide, calcolo la posizione
dello step successivo e poi impongo che i paracadute convergano verso la nuova posizione.

## Dinamica
### Modello 1
Modello lineare con $v_z$ controllabile:<br>
$\begin{cases}
x_{i+1}=x_i+v_x\Delta t + \nu_x\\
y_{i+1}=y_i+v_y\Delta t+ \nu_y\\
z_{i+1}=z_i+v_z\Delta t+ \nu_z\\
\end{cases}$<br>

$\begin{bmatrix}
x_{i+1}\\
y_{i+1}\\
z_{i+1}
\end{bmatrix}=\begin{bmatrix}
1&0&0\\
0&1&0\\
0&0&1
\end{bmatrix}\begin{bmatrix}
x_{i}\\
y_{i}\\
z_{i}
\end{bmatrix}+\begin{bmatrix}
\Delta t&0&0\\
0&\Delta t&0\\
0&0&\Delta t
\end{bmatrix}\begin{bmatrix}
v_x\\
v_y\\
v_z
\end{bmatrix}
+I \begin{bmatrix}\
\nu_x\\
\nu_y\\
\nu_z
\end{bmatrix}$ <br>

Controllo: LQR.

### Modello 2
Modello lineare con $v_z$ costante:<br>
$\begin{cases}
x_{i+1}=x_i+v_x\Delta t+ \nu_x\\
y_{i+1}=y_i+v_y\Delta t+\nu_y\\
z_{i+1}=z_i+\bar{v_z}\Delta t+\nu_z\\
\end{cases}$<br>

$\begin{bmatrix}
x_{i+1}\\
y_{i+1}\\
z_{i+1}
\end{bmatrix}=\begin{bmatrix}
1&0&0\\
0&1&0\\
0&0&1
\end{bmatrix}\begin{bmatrix}
x_{i}\\
y_{i}\\
z_{i}
\end{bmatrix}+\begin{bmatrix}
\Delta t&0\\
0&\Delta t\\
0&0
\end{bmatrix}
\begin{bmatrix}
v_x\\
v_y
\end{bmatrix}+
\begin{bmatrix}
1&0&0&0\\
0&1&0&0\\
0&0&1&\Delta t
\end{bmatrix}
\begin{bmatrix}\
\nu_x\\
\nu_y\\
\nu_z\\
\bar{v_z}
\end{bmatrix}$ <br>

Controllo: LQR con controllo solo su $v_x$ e $v_y$ .

### Modello 3
Non lineare con controllo su $v_x$ e $v_y$ , (nel frame locale) e rotazione (controllo su traslazione e
rotazione).
Controllo: o DDP o pseudo-LQR.<br>
$\begin{cases}
x_{i+1}=x_i+v_x\cos{\theta}\Delta t-v_y\sin{\theta}\Delta t+\nu_x\\
y_{i+1}=y_i+v_x\sin{\theta}\Delta t+v_y\cos{\theta}\Delta t+\nu_y\\
z_{i+1}=z_i+\bar{v_z}\Delta t+\nu_z\\
\theta_{i+1}=\theta_i+\omega \Delta t+\nu_{\theta}\\
\end{cases}$*<br>

La forma pseudo-lineare diventa:<br>

$\begin{bmatrix}
x_{i+1}\\
y_{i+1}\\
z_{i+1}\\
\theta_{i+1}
\end{bmatrix}=\begin{bmatrix}
1&0&0&0\\
0&1&0&0\\
0&0&1&0\\
0&0&0&1
\end{bmatrix}\begin{bmatrix}
x_{i}\\
y_{i}\\
z_{i}\\
\theta_i
\end{bmatrix}+\begin{bmatrix}
\cos{\theta}\Delta t&-\sin{\theta}\Delta t&0\\
\sin{\theta}\Delta t&\cos{\theta}\Delta t&0\\
0&0&0\\
0&0&\Delta t
\end{bmatrix}\begin{bmatrix}
v_x\\
v_y\\
\omega
\end{bmatrix}+
\begin{bmatrix}
1&0&0&0&0\\
0&1&0&0&0\\
0&0&1&\Delta t&0\\
0&0&0&0&1
\end{bmatrix}
\begin{bmatrix}
\nu_x\\
\nu_y\\
\nu_z\\
\bar{v_z}\\
\nu_{\theta}
\end{bmatrix}$<br>

Il sistema linearizzato (per DDP) è:

$A_t=\begin{bmatrix}
1&0&0&-v_x\sin{\theta}\Delta t-v_y\cos{\theta}\Delta t\\
0&1&0&v_x\cos{\theta}\Delta t-v_y\sin{\theta}\Delta t\\
0&0&1&0\\
0&0&0&1
\end{bmatrix}$

$B_t=\begin{bmatrix}
\cos{\theta}\Delta t&-\sin{\theta}\Delta t&0&0\\
\sin{\theta}\Delta t&\cos{\theta}\Delta t&0&0\\
0&0&\Delta t&0\\
0&0&0&\Delta t
\end{bmatrix}
$

### Modello 4:
Non lineare con velocità costante e controllo solo su $\theta$:<br>
$\begin{cases}
x_{i+1}=x_i-V\sin{\theta_i}\Delta t+ \nu_x\\
y_{i+1}=y_i+V\cos{\theta_i}\Delta t+ \nu_y\\
z_{i+1}=z_i+\bar{v_z}\Delta t+ \nu_z\\
\theta_{i+1}=\theta_i+\omega \Delta t+ \nu_{\theta}
\end{cases}$<br>

Per controllo: DDP o pseudo-LQR. <br>
Notare che, in realtà, il sistema è lineare rispetto a $\theta$ e $\omega$ e non lineare rispetto a $x, y, z$ e $V$.
Quindi l'LQR può essere tranquillamente usato rispetto a B.
La forma pseudo-lineare (per LQR) è:<br>

$\begin{bmatrix}
x_{i+1}\\
y_{i+1}\\
z_{i+1}\\
\theta_{i+1}
\end{bmatrix}=\begin{bmatrix}
1&0&0&0\\
0&1&0&0\\
0&0&1&0\\
0&0&0&1
\end{bmatrix} \begin{bmatrix}
x_{i}\\
y_{i}\\
z_{i}\\
\theta_i
\end{bmatrix}+\begin{bmatrix}
0\\
0\\
0\\
\Delta t
\end{bmatrix}
\omega +
\begin{bmatrix}
-\sin{\theta}\Delta t&1&0&0&0&0\\
\cos{\theta}\Delta t&0&1&0&0&0\\
0&0&0&1&\Delta t&0\\
0&0&0&0&0&1\\
\end{bmatrix} 
\begin{bmatrix}
V\\
\nu_x\\
\nu_y\\
\nu_z\\
\bar{v_z}\\
\nu_{\theta}
\end{bmatrix}$<br>

Il problema di questo modello dipende dal fatto che abbiamo il controllo solo su $\theta$, del quale non ci interessa alcun valore di convergenza, mentre ci interessa la convergenza di $x, y$, dei quali non abbiamo alcun controllo.<br>
Potendo solo far convergere $\theta$ a qualcosa, esso viene fatto convergere a un angolo tale per cui il paracadute punta sempre verso il target: infatti viene sospinto dal vento in direzione $y_{locale}$. <br>
Il suo moto sarà pertanto una spirale che si avvicina sempre più al target, ma non raggiunge mai il target. Questo è dovuto al fatto che il paracadute non può muoversi in direzione del target, ma solo ruotare su se stesso. <br>
Pertanto, $\theta_d$ non sarà zero ma un valore diverso, che dipende dalla posizione del paracadute rispetto al target: <br>

$\theta_d = \arctan{\frac{y_{locale}-y_{target}}{x_{locale}-x_{target}}} + \pi/2$ <br>
Il sistema linearizzato diventa:<br>

$A_t=
\begin{bmatrix}
1&0&0&-V\cos(\theta)\Delta t\\
0&1&0&-V\sin(\theta)\Delta t\\
0&0&1&0\\
0&0&0&1
\end{bmatrix}$

$B_t=
\begin{bmatrix}
0\\
0\\
0\\
\Delta t
\end{bmatrix}$

Bisogna ricavare la posizione dei paracadute i nel sistema di riferimento fisso (la terra) usando la posizione del paracadute j:<br>

$\begin{bmatrix}
x_i^g\\
y_i^g\\
z_i^g\\
1
\end{bmatrix}=
\begin{bmatrix}
\cos{\theta}&-\sin{\theta}&0&x_{i+1}-x_0\\
\sin{\theta}&\cos{\theta}&0&y_{i+1}-x_0\\
0&0&1&z_{i+1}-x_0\\
0&0&0&1\\
\end{bmatrix}
\begin{bmatrix}
x_j^g\\
y_j^g\\
z_j^g\\
1
\end{bmatrix}$

## Controllo
Partiamo con un controllo proporzionale calcolando l’angolo tra l’asse x locale del paracadute e
l’asse x del sistema fisso. Il PID andrebbe tarato per sistemi lineari, quindi per ora teniamo un
valore arbitrario per vedere se il KF funziona.

### PID
$$ u = k_p e_θ = k_p(π + \alpha − \theta) \\
\alpha = arctan( y − yt, x − xt)$$

### LQR

Il controllo tramite LQR si basa sulla minimizzazione della funzione costo quadratica:<br>

$J = \sum_{i=0}^{N-1} (x_i^T S x_i + u_i^T R u_i) + x_N^T S_f x_N$ <br>

dove $S$ e $R$ sono matrici di peso e $N$ è l’orizzonte di predizione. La soluzione è data da:<br>

$u = -Kx$<br>

dove $K$ è la matrice di guadagno calcolata risolvendo l’equazione di Riccati:<br>

$A^T P A − P + A^T P B (R + B^T P B)^{−1} B^T P A + Q = 0$<br>

con $P = P^T > 0$, da cui K è:<br>

$K = (R + B^T P B)^{−1} B^T P A$<br>

Le matrici $Q$ e $R$ sono matrici di peso che permettono di pesare i termini della funzione costo. Per
esempio, se vogliamo dare più importanza al controllo che allo stato, possiamo aumentare il peso
sulla matrice $R$. In questo modo, il controllo sarà più aggressivo e lo stato tenderà a seguire il
controllo. <br>
Se invece vogliamo dare più importanza allo stato, possiamo aumentare il peso sulla
matrice $Q$. In questo modo, il controllo sarà meno aggressivo e lo stato tenderà a seguire il
controllo. <br>
Se vogliamo dare più importanza allo stato finale, possiamo aumentare il peso sulla
matrice $S_f$. In questo modo, lo stato tenderà a seguire il controllo anche quando l’orizzonte di
predizione è finito.


### DDP
1. Dato $\bar{u}$, calcolare $\bar{x}$ tramite simulazione di dinamica $x^{+}=f(x,u)$
2. Backward Pass:<br>
**Add the matrices here**<br>
- I
- II for i = N-1,…, 0:
  - calcolare $Q_x, Q_u, Q_{xx}, Q_{uu}, Q_{xu}$
  -  matrix
  - calcolare  $V_{x}(i), V_{xx}(i)$
3. Forward Pass
 - I set $\alpha=1$
 - II simulate with $u = \bar{u} + \alpha \bar{w} + K(x - \bar{x})$
 - III if $\frac{J(U) - J(\bar{U})}{\Delta J(\alpha)}<c_1$, , decrease $\alpha$ and return to II, otherwise:
 - IV $\bar{x}=x, \bar{u}=u$
 - V if not converged: backward pass

Nel caso di dinamica NL, c’è il rischio che il paracadute esca dalla sua cella per raggiungere il nuovo centroide locale. Un modo per evitare ciò potrebbe essere quello di calcolare l’angolo tra il segmento congiungente il centroide locale con il paracadute e l’asse frontale del paracadute e farlo convergere a zero nel controllo ottimo.

## KALMAN FILTER
### Modello Lineare
Abbiamo la seguente dinamica:<br>
$x(t+1) = Ax(t) + Bu(t) + G\nu(t)$
con
- $\nu$ disturbi esterni (vento) a media nulla
- u input (velocità)
Nel KF inseriamo l’incertezza sull’attuazione e del disturbo:<br>
$ \bar{u} = u + \sigma\\
\bar{\nu} = \nu + \eta
$
e la predizione diventa:<br>
**Add the matrices here**<br>
L’errore attesa diventa:<br>
**Add the matrices here**<br>
da cui la matrice di covarianza:<br>
**Add the matrices here**<br>
con matrice di covarianza dell’input e matrice di covarianza del disturbo. I valori attesi combinati sono nulli poiché

### Distributed KF
Quando calcoliamo la posizione relativa tra due o più veicoli, tramite v2v communication, dato che la posizione stimata assoluta dell’altro veicolo è il risultato di un KF, avrà un andamento gaussiano ($\beta$). Il rumore di $\beta$ non è esattamente bianco, ma quasi (perché viene fuori da un KF) ma possiamo ignorarlo.
Il problema è che non possiamo calcolare la posizione relativa su entrambi i veicoli, perché la covarianza tra le due misure non sono più correlate: uno usa la posizione dell’altro.
Possiamo farlo usando la stima dello step precedente? Pare di sì.

### Distributed WLS
Quindi possiamo usare il consensus per:
- distribuire la posizione del centroide globale
- distribuire la stima delle posizioni di ogni paracadute
Con la seconda stima possiamo stimare anche implicitamente il centroide globale e conosciamo con più certezza la posizione dei paracadute vicini per fare la voronoi: infatti la misura relativa passa al consensus che migliora la stima. Mentre se distribuiamo il centroide, la misura relativa viene fatta una sola volta, quindi è più incerta. Infatti, se un drone non è raggiungibile da nessuno, la sua posizione viene randomizzato e questo incide sul calcolo del centroide, a meno che quando si fa il calcolo, quella non venga ignorato. In questo caso, però, la posizione del centroide non sarà accurata.
Scelta la seconda opzione, ci sono due modi che i paracadute hanno per comunicare la propria posizione:
1. Ogni paracadute comunica la propria posizione ai paracadute con cui riesce a comunicare, poi esegue una seconda comunicazione per farsi dire anche la posizione dei paracadute con cui non riesce a comunicare ma con cui altri hanno comunicato;
2. Ogni paracadute misura la posizione relativa dei paracadute vicini e poi esegue il consensus.
Il secondo modo ci permette di eseguire la stima della posizione dei paracadute vicini e della propria, usando più informazioni possibili, mentre nel primo metodo le posizioni dei paracadute non vengono mai sottoposte a una algoritmo di stima: abbiamo una sola misura invece che n misure.
Un problema sorge se un paracadute è completamente isolato e nessuno può comunicare con lui. In quel caso la posizione random ipotizzata deve essere il più possibile lontana dalla posizione attuale del paracadute che la calcola, in modo tale che durante la voronoi tasselation non venga considerato nel sensing range. Quindi, se un paracadute non ne vede uno, lo mette lontano e mette la covarianza alta. Quando poi calcolo la posizione del centroide globale, alla fine del consensus, taglio fuori i paracadute con posizione incerta sopra una certa soglia (cioè quelli che non sono mai entrati nello consensus), in modo tale da calcolare il centroide globale sui paracadute nella cerchia di conoscenze. I paracadute tagliati fuori si calcoleranno il centroide globale su se stessi, non potendo comunicare con nessuno, pertanto si creano diverse dinamiche, che possono favorire un riavvicinamento dei paracadute tagliati fuori al resto dello stormo.
Altrimenti si può usare una media pesata sulla incertezza della posizione degli altri paracadute, in modo tale da non buttare via nessuna informazione. Il risultato è molto simile.

Attenzione: dopo aver fatto il WLS il paracadute i non può aggiornare la sua posizione con le informazioni ricevute dagli altri, ma deve usare la sua precedente al conensus. Questo problema sorge dal fatto che l'uscita di un KF non può essere usata come input in un altro. In particoalre, ogni paracadute stima la sua posizione con il proprio KF e la posizione degli altri sommando alla propria posizione stimata la misura relativa. Quando viene fatto il consenso, al paracadute i arrivano le posizioni degli altri N-1, che a loro volta sono date dalla combinazione degli N-1 KF e misure relative. Ciò significa che alla fine del consenso la posizione che il robot i conosce di se stesso dipende in qualche modo dagli altri N-1 KF, e perciò allo step successivo non può essere utilizzata come prior nel KF. <br>
Questo fatto fa si che in tutte le parti di codice dove l'agente i debba usare la sua posizione (ie Voronoi tessellation, controllo a basso livello) devono essere fatti con il valore della posizione prima del WLS.

### Voronoi Tesselation
Il controllo andrà fatto sul centroide dello stormo di paracaduti, che a sua volta sarà una funzione delle posizioni dei paracadute.
Inoltre, introduciamo la Voronoi tasselation per evitare che, tra uno step e l’altro, durante la fase di controllo e spostamento, non ci siano scontri tra paracadute: viene calcolato tramite controllo ottimo la nuova posizione del centroide globale, modellato come un drone fittizio, con una sua dinamica e un input fittizio. Una volta calcolata la posizione futura desiderata del centroide globale col controllo ottimo, tutti gli altri centroidi devono spostarsi in modo da realizzare quello spostamento. Si calcola il vettore spostamento del centroide globale, quindi si applica ad ogni drone. Viene posta una gaussiana su ogni nuovo centroide desiderato in modo da spostare il centroide di ogni paracadute. Per spostare poi il paracadute si implementa un controllo proporzionale verso il nuovo centroide.
Per garantire l’assenza di scontri, dato che i droni in realtà non sono punti ma hanno delle dimensioni, si aggiungono modifiche alla Voronoi tassellation (slide 51).
Infine, si può unire la Voronoi tasselation con il sensing range per diminuire l’area della cella

#### Vettore spostamento
Per calcolare la posizione che uno stormo di droni deve raggiungere per spostare il centroide del gruppo in una posizione desiderata, si possono seguire i seguenti passaggi:
1. Misurare la posizione attuale del centroide dello stormo.
2. Calcolare la differenza tra la posizione attuale del centroide e la posizione desiderata del centroide. Questa differenza è il vettore di spostamento che indica la direzione e la distanza che lo stormo deve percorrere per raggiungere la posizione desiderata del centroide.
3. Dividere il vettore di spostamento per il numero di droni nello stormo per ottenere il vettore di spostamento che ogni drone deve effettuare per spostare lo stormo nella posizione desiderata del centroide.
4. Aggiungere il vettore di spostamento individuale di ogni drone alla sua posizione attuale per calcolare la nuova posizione in cui ogni drone deve volare per spostare lo stormo nella posizione desiderata del centroide

#### Funzione di Densità 
Un altro modo è quello di usare una funzione di densità gaussiana con media nel target, di modo che il nuovo centroide locale calcolato punti in quella direzione: in questo modo non c’è bisogno di introdurre il centroide globale, poiché ogni drone tenderà atonomamente a raggiungere il target mantenendo le distanze grazie a Voronoi.
Il problema di questo approccio è che non c’è nulla che raggiunga il target, né i droni né un punto geometrico (centroide globale).

## Workflow
1. Inizializzazione paracadute
2. Loop nel tempo
  - I. Localizzazione
    - a. Simulare misura di posizione di se stessi
    - b. Kalman filter sulla posizione
    - c. Simulare misura di posizione degli altri
  - II. Distribuzione informazioni (consensus+centroide)
  - III. Voronoi tessellation
  - IV. Plot dinamici (devono essere fatti qui perchè altrimenti successivaente la posizione risulta essere aggiornata mentre la tassellazione no)
  - V. Dinamica:
    - a. Calcolo traiettoria ottima del centroide globale
    - b. Calcolo nuova posizione paracadute
    - c. Posizionare gaussiana sulla nuova posizione
    - d. Calcolo nuovo centroide locale
    - e. Controllo a basso livello
3. Plots traiettorie finali

# Cose da fare
- Mettere vincolo di voronoi sul cerchi esterno
- Considerare la adjecy matrix non simmetrica (i.e. grafi diretti) quando si fa il consenso sulla posizione del centroide globale
- Considerare un valore sensato per la covarainza nella stima distribuita del centroide globale
- Decidere il numero massimo di messaggi m che possono essere scambiati compatibile con la lunghezza di un time step. Vedere se introdurre una probabilità di scambio di informazione quando si fa il consensus
- Cambiare i commenti al codice nella funzione di voronoi
- Studiare i fondamenti teorici che assicurano/non assicurano la convergenza del centroide stimato verso quello vero 
- Controllore di basso liello che faccia muovere il robot solamente all'interno della cella (nel caso di un robot con dinamica non lineare)
- Mettere valore sensato per R_relative in intializiation
- Fare in modo che la distanza verticale per cui si decide o meno di considerare un paracadute nella voronoi sia leggermente maggiore della vera dimensione del paracadute