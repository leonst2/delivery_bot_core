//! Optimal assignment (Hungarian / Kuhn-Munkres, O(n^2 m)). Entries that are not finite are forbidden pairs.
//! Ties are broken by the lowest index, so the result is deterministic.

const FORBIDDEN: f64 = 1e9;

/// For every row the assigned column, or `None` when no allowed column is left for it.
pub fn assign(cost: &[Vec<f64>]) -> Vec<Option<usize>> {
    let n = cost.len();
    let m = cost.first().map_or(0, Vec::len);
    if n == 0 || m == 0 {
        return vec![None; n];
    }
    if n > m {
        let transposed: Vec<Vec<f64>> = (0..m)
            .map(|j| (0..n).map(|i| cost[i][j]).collect())
            .collect();
        let mut rows = vec![None; n];
        for (j, i) in assign(&transposed).into_iter().enumerate() {
            if let Some(i) = i {
                rows[i] = Some(j);
            }
        }
        return rows;
    }
    let a = |i: usize, j: usize| {
        let c = cost[i][j];
        if c.is_finite() {
            c
        } else {
            FORBIDDEN
        }
    };
    // 1-based potentials and matching as in the classic formulation; column 0 is a sentinel.
    let (mut u, mut v) = (vec![0.0; n + 1], vec![0.0; m + 1]);
    let (mut p, mut way) = (vec![0usize; m + 1], vec![0usize; m + 1]);
    for i in 1..=n {
        p[0] = i;
        let mut j0 = 0;
        let mut minv = vec![f64::INFINITY; m + 1];
        let mut used = vec![false; m + 1];
        loop {
            used[j0] = true;
            let i0 = p[j0];
            let (mut delta, mut j1) = (f64::INFINITY, 0);
            for j in 1..=m {
                if used[j] {
                    continue;
                }
                let cur = a(i0 - 1, j - 1) - u[i0] - v[j];
                if cur < minv[j] {
                    minv[j] = cur;
                    way[j] = j0;
                }
                if minv[j] < delta {
                    delta = minv[j];
                    j1 = j;
                }
            }
            for j in 0..=m {
                if used[j] {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
            if p[j0] == 0 {
                break;
            }
        }
        loop {
            let j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
            if j0 == 0 {
                break;
            }
        }
    }
    let mut rows = vec![None; n];
    for j in 1..=m {
        if p[j] != 0 && cost[p[j] - 1][j - 1].is_finite() {
            rows[p[j] - 1] = Some(j - 1);
        }
    }
    rows
}

#[cfg(test)]
mod tests {
    use super::*;

    const X: f64 = f64::INFINITY;

    #[test]
    fn finds_the_optimum_not_the_greedy_choice() {
        // Greedy would take (0, 0) = 1 and then (1, 1) = 10: total 11. Optimal is 2 + 3 = 5.
        let cost = vec![vec![1.0, 2.0], vec![3.0, 10.0]];
        assert_eq!(assign(&cost), vec![Some(1), Some(0)]);
    }

    #[test]
    fn handles_rectangles_and_forbidden_pairs() {
        let cost = vec![vec![X, 1.0, X], vec![X, X, X]];
        assert_eq!(assign(&cost), vec![Some(1), None]);
        let tall = vec![vec![5.0], vec![1.0], vec![X]];
        assert_eq!(assign(&tall), vec![None, Some(0), None]);
        assert_eq!(assign(&[vec![], vec![]]), vec![None, None]);
        assert!(assign(&[]).is_empty());
    }

    #[test]
    fn ties_go_to_the_lowest_index() {
        let cost = vec![vec![1.0, 1.0], vec![1.0, 1.0]];
        assert_eq!(assign(&cost), vec![Some(0), Some(1)]);
    }
}
