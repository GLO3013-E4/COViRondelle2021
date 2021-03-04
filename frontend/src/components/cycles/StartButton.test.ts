import wrapWithVuetifyAndStore from '@/util/wrapWithVuetifyAndStore';
import StartButton from '@/components/cycles/StartButton.vue';

describe('When mounting StartButton component', () => {
  const wrapper = wrapWithVuetifyAndStore(StartButton);

  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});
