#!/usr/bin/env python3

import matplotlib.pyplot as plt


LOOKAHEAD_GAINS = [0.5, 1.0, 1.5]

MEAN_ERRORS = [0.1690, 0.0539, 0.0077]

MAX_ERRORS = [0.5085, 0.2555, 0.0454]


def main():
    """Plot Pure Pursuit lookahead tuning results."""
    plt.figure()

    plt.plot(
        LOOKAHEAD_GAINS,
        MEAN_ERRORS,
        marker='o',
        label='Mean cross-track error'
    )

    plt.plot(
        LOOKAHEAD_GAINS,
        MAX_ERRORS,
        marker='s',
        label='Maximum cross-track error'
    )

    plt.xlabel('Lookahead gain k')
    plt.ylabel('Cross-track error [m]')
    plt.title('Pure Pursuit Lookahead Gain Tuning')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    output_file = 'data/lookahead_tuning.png'

    plt.savefig(
        output_file,
        dpi=300
    )

    plt.close()

    print(
        f'Tuning plot saved to: {output_file}'
    )


if __name__ == '__main__':
    main()
