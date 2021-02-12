import StationInformation from '@/components/station/StationInformation.vue';
import useVuetify from '@/hooks/useVuetify';

const wrapper = useVuetify(StationInformation);

describe('When mounting station information', () => {
  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});
