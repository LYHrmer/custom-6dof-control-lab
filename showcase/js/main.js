/* Native video controls; timestamps are video positions, never robot measurements. */
(() => {
  const video = document.querySelector('#demo-video');
  const playOverlay = document.querySelector('#video-start');
  const status = document.querySelector('#player-status');
  const error = document.querySelector('#media-error');
  const moments = [...document.querySelectorAll('[data-seek]')];
  const rates = [...document.querySelectorAll('[data-rate]')];
  const reducedMotion = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
  const time = value => {
    const seconds = Number.isFinite(value) ? Math.max(0, Math.floor(value)) : 0;
    return `${String(Math.floor(seconds / 60)).padStart(2, '0')}:${String(seconds % 60).padStart(2, '0')}`;
  };

  async function play(at) {
    if (Number.isFinite(at)) {
      if (video.readyState < 1) {
        video.addEventListener('loadedmetadata', () => play(at), {once: true});
        video.load();
        return;
      }
      video.currentTime = Math.max(0, Math.min(at, video.duration));
    }
    try {
      await video.play();
      status.textContent = '正在播放实物录像';
    } catch (reason) {
      status.textContent = '暂时无法自动播放，请使用视频原生播放按钮。';
    }
  }
  document.querySelector('#hero-play').addEventListener('click', () => {
    play();
    video.scrollIntoView({behavior: reducedMotion ? 'auto' : 'smooth', block: 'center'});
  });
  playOverlay.addEventListener('click', () => play(video.ended ? 0 : undefined));
  document.querySelector('#replay').addEventListener('click', () => play(0));
  rates.forEach(button => button.addEventListener('click', () => {
    video.playbackRate = Number(button.dataset.rate);
    rates.forEach(rate => rate.setAttribute('aria-pressed', String(rate === button)));
    status.textContent = `播放速度 ${video.playbackRate} 倍`;
  }));
  moments.forEach(button => button.addEventListener('click', () => {
    play(Number(button.dataset.seek));
    video.scrollIntoView({behavior: reducedMotion ? 'auto' : 'smooth', block: 'center'});
  }));
  video.addEventListener('play', () => { playOverlay.hidden = true; });
  video.addEventListener('ended', () => { playOverlay.hidden = false; playOverlay.setAttribute('aria-label', '重新播放实物演示'); });
  video.addEventListener('loadedmetadata', () => {
    document.querySelector('#duration').textContent = time(video.duration);
    error.hidden = true;
  });
  video.addEventListener('timeupdate', () => {
    document.querySelector('#current-time').textContent = time(video.currentTime);
    const active = [...moments].reverse().find(button => video.currentTime >= Number(button.dataset.seek));
    moments.forEach(button => {
      button.classList.toggle('active', button === active);
      if (button === active) button.setAttribute('aria-current', 'true');
      else button.removeAttribute('aria-current');
    });
  });
  function showMediaError() {
    error.hidden = false;
    playOverlay.hidden = true;
    document.querySelector('#hero-play').disabled = true;
    document.querySelector('#replay').disabled = true;
    moments.forEach(button => { button.disabled = true; });
  }
  video.addEventListener('error', showMediaError);
  video.querySelector('source').addEventListener('error', showMediaError);
  document.querySelector('#fullscreen').addEventListener('click', async () => {
    try {
      if (video.requestFullscreen) await video.requestFullscreen();
      else if (video.webkitEnterFullscreen) video.webkitEnterFullscreen();
      else status.textContent = '此浏览器请使用视频原生全屏按钮。';
    } catch { status.textContent = '请使用视频原生全屏按钮。'; }
  });

  // Window-based adaptation of the web-builder starter's navigation/reveal pattern.
  const links = [...document.querySelectorAll('.nav-link')];
  const sections = [...document.querySelectorAll('section[data-section]')];
  let ticking = false;
  function updateNavigation() {
    let current = sections[0].id;
    sections.forEach(section => { if (section.getBoundingClientRect().top <= 150) current = section.id; });
    links.forEach(link => {
      const active = link.dataset.section === current;
      link.classList.toggle('active', active);
      if (active) link.setAttribute('aria-current', 'location');
      else link.removeAttribute('aria-current');
    });
    ticking = false;
  }
  window.addEventListener('scroll', () => {
    if (!ticking) { ticking = true; requestAnimationFrame(updateNavigation); }
  }, {passive: true});
  updateNavigation();
  if ('IntersectionObserver' in window && !reducedMotion) {
    document.documentElement.classList.add('js-ready');
    const observer = new IntersectionObserver(entries => entries.forEach(entry => {
      if (entry.isIntersecting) { entry.target.classList.add('in-view'); observer.unobserve(entry.target); }
    }), {threshold: 0.08});
    document.querySelectorAll('.reveal').forEach(element => observer.observe(element));
  }
})();
