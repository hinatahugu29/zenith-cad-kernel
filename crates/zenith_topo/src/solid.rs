use crate::shell::{Shell, ShellValidationReport};
use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicU64, Ordering};
use zenith_math::Tolerance;

static SOLID_ID_GEN: AtomicU64 = AtomicU64::new(1);

/// B-Rep ソリッド（Solid: 閉じた外側シェル + 内部空洞シェル）
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Solid {
    pub id: u64,
    pub outer_shell: Shell,
    pub inner_shells: Vec<Shell>,
}

/// Solid の検証失敗内容
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SolidValidationError {
    pub outer_shell_report: ShellValidationReport,
    pub inner_shell_reports: Vec<ShellValidationReport>,
}

impl std::fmt::Display for SolidValidationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let outer_count = self.outer_shell_report.errors.len();
        let inner_count: usize = self
            .inner_shell_reports
            .iter()
            .map(|report| report.errors.len())
            .sum();
        let outer_err_str = self.outer_shell_report.errors.join("; ");
        write!(
            f,
            "Solid validation failed with {outer_count} outer-shell errors ({outer_err_str}) and {inner_count} inner-shell errors"
        )
    }
}


impl std::error::Error for SolidValidationError {}

impl Solid {
    pub fn new(outer_shell: Shell, inner_shells: Vec<Shell>) -> Self {
        assert!(
            outer_shell.is_closed,
            "Outer shell of a Solid must be closed"
        );
        for s in &inner_shells {
            assert!(s.is_closed, "Inner shell (cavity) must be closed");
        }
        Self {
            id: SOLID_ID_GEN.fetch_add(1, Ordering::Relaxed),
            outer_shell,
            inner_shells,
        }
    }

    /// 中空なしの単純ソリッド
    pub fn simple(outer_shell: Shell) -> Self {
        Self::new(outer_shell, Vec::new())
    }

    /// 検証付きでSolidを作成する。
    pub fn try_new(
        outer_shell: Shell,
        inner_shells: Vec<Shell>,
        tol: &Tolerance,
    ) -> Result<Self, SolidValidationError> {
        let outer_shell_report = outer_shell.validate_closed(tol);
        let inner_shell_reports: Vec<ShellValidationReport> = inner_shells
            .iter()
            .map(|shell| shell.validate_closed(tol))
            .collect();

        let valid =
            outer_shell_report.is_valid() && inner_shell_reports.iter().all(|r| r.is_valid());
        if !valid {
            return Err(SolidValidationError {
                outer_shell_report,
                inner_shell_reports,
            });
        }

        Ok(Self::new(outer_shell, inner_shells))
    }

    /// 中空なしの単純ソリッドを検証付きで作成する。
    pub fn try_simple(outer_shell: Shell, tol: &Tolerance) -> Result<Self, SolidValidationError> {
        Self::try_new(outer_shell, Vec::new(), tol)
    }

    /// 外側・内側シェルが閉じたソリッドとして妥当か検証する。
    pub fn is_topologically_valid(&self, tol: &Tolerance) -> bool {
        self.outer_shell.is_topologically_closed(tol)
            && self
                .inner_shells
                .iter()
                .all(|shell| shell.is_topologically_closed(tol))
    }
}
