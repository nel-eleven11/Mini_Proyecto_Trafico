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
gcc -O2 -fopenmp -std=c11 traffic_omp.c -o traffic_omp
```

#### Correr:

- v: número de vehículos
- p: número de pasos

Secuencial:

```bash
./traffic_seq v p
```

Paralelo:

```bash
OMP_NUM_THREADS=4 ./traffic_omp v p
```
