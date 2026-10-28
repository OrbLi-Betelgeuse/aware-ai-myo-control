// Keep demo controls and the original live telemetry in one flowing column.
(() => {
  const params = new URLSearchParams(location.search);
  if (params.get('controller') !== '3.2a' ||
      (params.get('input') !== 'native' && params.get('keyboard') !== '1')) return;

  function mount() {
    const panel = document.getElementById('keyboard-demo-panel');
    const hud = document.getElementById('hud');
    if (!panel || !hud) return false;
    const dock = document.createElement('div');
    dock.id = 'demo-status-dock';
    Object.assign(dock.style, {
      position: 'fixed', left: '12px', bottom: '12px', zIndex: '40',
      display: 'flex', flexDirection: 'column', gap: '8px',
      width: 'min(430px, calc(50vw - 24px))', overflowY: 'auto',
      overflowX: 'hidden', scrollbarGutter: 'stable',
      overscrollBehavior: 'contain'
    });
    for (const child of [panel, hud]) {
      Object.assign(child.style, {
        position: 'relative', left: 'auto', bottom: 'auto',
        width: '100%', minWidth: '0', maxWidth: 'none',
        boxSizing: 'border-box', flexShrink: '0', overflowWrap: 'anywhere'
      });
      dock.appendChild(child);
    }
    document.body.appendChild(dock);
    function resize() {
      const connection = document.getElementById('connection');
      const top = connection ? connection.getBoundingClientRect().bottom + 12 : 12;
      dock.style.maxHeight = Math.max(0, window.innerHeight - top - 12) + 'px';
    }
    resize();
    window.addEventListener('resize', resize);
    const connection = document.getElementById('connection');
    if (connection && typeof ResizeObserver !== 'undefined') {
      new ResizeObserver(resize).observe(connection);
    }
    return true;
  }
  if (!mount()) {
    // The simulator module can finish loading after the demo scripts.
    const observer = new MutationObserver(() => {
      if (mount()) observer.disconnect();
    });
    observer.observe(document.body, {childList: true, subtree: true});
  }
})();
