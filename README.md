# TE3001B_TEAM_3
TE3001B Team 3 Repository

## Gráficas de xarm_perturbations

Las gráficas generadas por el paquete `xarm_perturbations` se encuentran en:

```
src/xarm_perturbations/results/
├── baseline_20260224_201030/       # Sin perturbaciones
│   ├── position_tracking.png
│   ├── position_error.png
│   ├── velocity_magnitude.png
│   ├── metrics.txt
│   └── tracking_data.csv
│
├── gaussian_20260224_203623/       # Perturbación gaussiana
│   ├── position_tracking.png
│   ├── position_error.png
│   ├── velocity_magnitude.png
│   ├── metrics.txt
│   └── tracking_data.csv
│
└── sine_20260224_213246/           # Perturbación senoidal
    ├── position_tracking.png
    ├── position_error.png
    ├── velocity_magnitude.png
    ├── metrics.txt
    └── tracking_data.csv
```

Cada carpeta corresponde a un escenario de prueba y contiene tres gráficas:
- **position_tracking.png** – Trayectoria deseada vs. trayectoria real del brazo.
- **position_error.png** – Error de posición a lo largo del tiempo.
- **velocity_magnitude.png** – Magnitud de velocidad durante la ejecución.

Las gráficas son generadas automáticamente por el nodo `evaluator.py` al finalizar cada prueba.
