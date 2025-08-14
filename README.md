# Mini_Proyecto_Trafico
Simulación de tráfico utilizando OpenMP para paralelizar operaciones

---

## Compilar con:


#### Secuencial:

```bash
gcc -O2 -std=c11 traffic_seq.c -o traffic_seq
```

Paralelo:

```bash
gcc -O3 -march=native -fopenmp -std=c11 traffic_omp.c -o traffic_omp
```

#### Correr:

- v: número de vehículos
- t: tiempo de impresión de iteraciones

Secuencial:

```bash
./traffic_seq v t
```

Paralelo:

```bash
OMP_NUM_THREADS=8 OMP_PROC_BIND=spread OMP_PLACES=cores ./traffic_omp v t
```
